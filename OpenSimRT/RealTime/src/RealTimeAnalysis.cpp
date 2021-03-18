/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 */
#include "RealTimeAnalysis.h"
#include "Exception.h"
#include "JointReaction.h"
#include <SimTKcommon/internal/BigMatrix.h>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

void RealTimeAnalysis::FilteredData::fromVector(const double& time,
                                                const SimTK::Vector& x,
                                                const SimTK::Vector& xd,
                                                const SimTK::Vector& xdd,
                                                const int& nq) {
    this->t = time;
    this->q = x(0, nq);
    this->qd = xd(0, nq);
    this->qdd = xdd(0, nq);
    this->externalWrenches = {};

    auto wrenchSize = ExternalWrench::Input::size();
    int wrenchCount = (x.size() - nq) / wrenchSize;

    ExternalWrench::Input wrench;
    for (int i = 0; i < wrenchCount; ++i) {
        int offset = i * wrenchSize + nq;
        wrench.fromVector(x(offset, offset + wrenchCount));
        this->externalWrenches.push_back(wrench);
    }
}

RealTimeAnalysis::RealTimeAnalysis(
        const Model& otherModel, const RealTimeAnalysis::Parameters& parameters)
        : model(*otherModel.clone()), parameters(parameters),
          previousAcquisitionTime(-1.0), previousProcessingTime(-1.0),
          notifyParentThread(false), terminationFlag(false) {
    // filter
    lowPassFilter = new LowPassSmoothFilter(parameters.filterParameters);

    // ik
    inverseKinematics = new InverseKinematics(
            model, parameters.ikMarkerTasks, parameters.ikIMUTasks,
            parameters.ikConstraintsWeight, parameters.ikAccuracy);

    // id
    inverseDynamics = new InverseDynamics(model, parameters.wrenchParameters);

    // so
    muscleOptimization = new MuscleOptimization(
            model, parameters.muscleOptimizationParameters,
            parameters.momentArmFunction);

    // jr
    jointReaction = new JointReaction(model, parameters.wrenchParameters);
}

bool RealTimeAnalysis::shouldTerminate() { return terminationFlag.load(); }

void RealTimeAnalysis::shouldTerminate(bool flag) { terminationFlag = flag; }

void RealTimeAnalysis::run() {
    thread acquisitionThread(&RealTimeAnalysis::acquisition, this);
    thread processingThread(&RealTimeAnalysis::processing, this);
    acquisitionThread.detach();
    processingThread.detach();
}

Vector RealTimeAnalysis::prepareUnfilteredData(
        const Vector& q,
        const vector<ExternalWrench::Input>& externalWrenches) const {
    int m = q.size() + externalWrenches.size() * ExternalWrench::Input::size();
    if (m == 0) { THROW_EXCEPTION("cannot convert from empty"); }

    Vector v(m);
    v(0, q.size()) = q;
    for (int i = 0; i < externalWrenches.size(); ++i) {
        auto w = externalWrenches[i].toVector();
        v(q.size() + i * w.size(), w.size()) = w;
    }
    return v;
}

void RealTimeAnalysis::acquisition() {
    try {
        while (true) {
            if (shouldTerminate()) THROW_EXCEPTION("Acquisition terminated.");

            // get data
            auto acquisitionData = parameters.dataAcquisitionFunction();
            if (previousAcquisitionTime >= acquisitionData.IkFrame.t) {
                continue;
            }

            // update time
            previousAcquisitionTime = acquisitionData.IkFrame.t;

            // perform ik
            auto pose = inverseKinematics->solve(acquisitionData.IkFrame);

            // filter
            auto unfilteredData = prepareUnfilteredData(
                    pose.q, acquisitionData.ExternalWrenches);
            auto filteredData = lowPassFilter->filter({pose.t, unfilteredData});

            // push to buffer
            if (!filteredData.isValid) continue;
            buffer.add(filteredData);
        }
    } catch (exception& e) {
        cout << e.what() << endl;

        // raise termination flag
        terminationFlag = true;

        // notify buffer to not get stuck in processing thread
        buffer.setDataRetrievalMode(DataRetrievalMode::CONTINUOUS);
    }
}

void RealTimeAnalysis::processing() {
    try {
        FilteredData filteredData;
        Vector am, fm, residuals, reactionWrenchVector;
        Vector_<SpatialVec> reactionWrenches;
        while (true) {
            if (shouldTerminate()) THROW_EXCEPTION("Processing terminated.");

            // get data from buffer
            auto data = buffer.get(1)[0];
            filteredData.fromVector(data.t, data.x, data.xDot, data.xDDot,
                                    model.getNumCoordinates());

            // solve id
            auto id = inverseDynamics->solve({filteredData.t, filteredData.q,
                                              filteredData.qd, filteredData.qdd,
                                              filteredData.externalWrenches});

            // solve so and jr
            if (parameters.solveMuscleOptimization) {
                auto so = muscleOptimization->solve(
                        {filteredData.t, filteredData.q, id.tau});
                am = so.am;
                fm = so.fm;
                residuals = so.residuals;

                auto jr = jointReaction->solve({filteredData.t, filteredData.q,
                                                filteredData.qd, so.fm,
                                                filteredData.externalWrenches});
                reactionWrenches = jr.reactionWrench;
                reactionWrenchVector = jointReaction->asForceMomentPoint(jr);
            }

            { // thread-safe write to output
                lock_guard<mutex> locker(mu);
                output.t = filteredData.t;
                output.q = filteredData.q;
                output.qd = filteredData.qd;
                output.qdd = filteredData.qdd;
                output.grfRightWrench =
                        filteredData.externalWrenches[0].toVector();
                output.grfLeftWrench =
                        filteredData.externalWrenches[1].toVector();
                output.tau = id.tau;
                output.am = am;
                output.fm = fm;
                output.residuals = residuals;
                output.reactionWrenches = reactionWrenches;
                output.reactionWrenchVector = reactionWrenchVector;
            }
            // notify main thread to read output
            notifyParentThread = true;
            cond.notify_one();
        }
    } catch (const std::exception& e) {
        cout << e.what() << endl;

        // raise termination flag
        terminationFlag = true;

        // notify main thread in case of exception
        notifyParentThread = true;
        cond.notify_one();
    }
}

RealTimeAnalysis::Output RealTimeAnalysis::getResults() {
    unique_lock<mutex> locker(mu);
    cond.wait(locker, [&]() { return notifyParentThread.load(); });
    notifyParentThread = false;
    return output;
}

RealTimeAnalysis::Loggers RealTimeAnalysis::initializeLoggers() {
    if (!inverseKinematics)
        THROW_EXCEPTION("InverseKinematics object hasn't been instantiated.");
    if (!inverseDynamics)
        THROW_EXCEPTION("InverseDynamics object hasn't been instantiated.");
    if (!muscleOptimization)
        THROW_EXCEPTION("MuscleOptimization object hasn't been instantiated.");
    if (!jointReaction)
        THROW_EXCEPTION("JointReaction object hasn't been instantiated.");

    // ik
    log.qLogger = inverseKinematics->initializeLogger();
    log.qDotLogger = inverseKinematics->initializeLogger();
    log.qDDotLogger = inverseKinematics->initializeLogger();

    // id
    log.tauLogger = inverseDynamics->initializeLogger();

    // so
    log.fmLogger = muscleOptimization->initializeMuscleLogger();
    log.amLogger = muscleOptimization->initializeMuscleLogger();
    log.residualLogger = muscleOptimization->initializeMuscleLogger();

    // jr
    log.jrLogger = jointReaction->initializeLogger();

    return log;
}
