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
#include "RealTimeAnalysisLegacy.h"
#include "CircularBuffer.h"
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Common/Signal.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

Vector RealTimeAnalysisLegacy::UnfilteredData::toVector() {
    if (size() == 0) { THROW_EXCEPTION("cannot convert from empty"); }

    Vector temp(size());
    int index = 0;
    temp[index++] = t;
    for (int i = 0; i < q.size(); ++i) { temp[index++] = q[i]; }

    for (int i = 0; i < externalWrenches.size(); ++i) {
        auto tempVec = externalWrenches[i].toVector();
        for (int j = 0; j < tempVec.size(); ++j) { temp[index++] = tempVec[j]; }
    }

    return temp;
}

int RealTimeAnalysisLegacy::UnfilteredData::size() {
    return 1 + q.size() +
           externalWrenches.size() * ExternalWrench::Input::size();
}

RealTimeAnalysisLegacy::RealTimeAnalysisLegacy(
        const Model& otherModel,
        const RealTimeAnalysisLegacy::Parameters& parameters)
        : parameters(parameters), previousAcquisitionTime(-1.0) {
    inverseKinematics = new InverseKinematics(
            otherModel, parameters.ikMarkerTasks, parameters.ikIMUTasks,
            parameters.ikConstraintsWeight, parameters.ikAccuracy);

    inverseDynamics =
            new InverseDynamics(otherModel, parameters.wrenchParameters);

    muscleOptimization = new MuscleOptimization(
            otherModel, parameters.muscleOptimizationParameters,
            parameters.momentArmFunction);

    jointReaction = new JointReaction(otherModel, parameters.wrenchParameters);

    // visualization
    if (parameters.useVisualizer) {
        visualizer = new BasicModelVisualizer(otherModel);
        for (int i = 0; i < parameters.wrenchParameters.size(); ++i) {
            GRFDecorators.push_back(new ForceDecorator(Green, 0.001, 3));
            visualizer->addDecorationGenerator(GRFDecorators.back());
        }
        for (int i = 0; i < parameters.reactionForceOnBodies.size(); ++i) {
            reactionForceDecorators.push_back(
                    new ForceDecorator(Red, 0.0005, 3));
            visualizer->addDecorationGenerator(reactionForceDecorators.back());
        }
    }

    // loggers
    // qFiltered = inverseKinematics->initializeLogger();
    // qDotFiltered = inverseKinematics->initializeLogger();
    // qDDotFiltered = inverseKinematics->initializeLogger();
}

void RealTimeAnalysisLegacy::run() {
    thread acquisitionThread(&RealTimeAnalysisLegacy::acquisition, this);
    thread processingThread(&RealTimeAnalysisLegacy::processing, this);
    acquisitionThread.join();
    processingThread.join();
}

void RealTimeAnalysisLegacy::acquisition() {
    while (!visualizer->getShouldTerminate()) {
        try {
            // get data from Vicon or other method
            auto acquisitionData = parameters.dataAcquisitionFunction();
            if (previousAcquisitionTime >= acquisitionData.ikFrame.t) {
                continue;
            }

            cout << "acquisition: " << acquisitionData.ikFrame.t << endl;

            // perform IK
            auto pose = inverseKinematics->solve(acquisitionData.ikFrame);

            // push to buffer
            UnfilteredData unfilteredData;
            unfilteredData.t = pose.t;
            unfilteredData.q = pose.q;
            unfilteredData.externalWrenches = acquisitionData.externalWrenches;
            buffer.add(unfilteredData);

            // update time
            previousAcquisitionTime = pose.t;
        } catch (exception& e) {
            cout << e.what() << endl;
            cout << "acquisition terminated" << endl;
            visualizer->setShouldTerminate(true);
        }
    }
}

void RealTimeAnalysisLegacy::processing() {
    while (!visualizer->getShouldTerminate()) {
        auto filteredData = filterData();
        Vector am;
        Vector_<SpatialVec> reactionWrenches;

        cout << "Processing: " << filteredData.t << endl;

        // solve id
        auto id = inverseDynamics->solve({filteredData.t, filteredData.q,
                                          filteredData.qd, filteredData.qdd,
                                          filteredData.externalWrenches});

        if (parameters.solveMuscleOptimization) {
            auto so = muscleOptimization->solve(
                    {filteredData.t, filteredData.q, id.tau});
            am = so.am;

            auto jr = jointReaction->solve({filteredData.t, filteredData.q,
                                            filteredData.qd, so.fm,
                                            filteredData.externalWrenches});
            reactionWrenches = jr.reactionWrench;
        }

        // visualization
        if (parameters.useVisualizer) {
            visualizer->update(filteredData.q, am);
            for (int i = 0; i < filteredData.externalWrenches.size(); ++i) {
                GRFDecorators[i]->update(
                        filteredData.externalWrenches[i].point,
                        filteredData.externalWrenches[i].force);
            }
            for (int i = 0; i < parameters.reactionForceOnBodies.size(); ++i) {
                visualizer->updateReactionForceDecorator(
                        reactionWrenches, parameters.reactionForceOnBodies[i],
                        reactionForceDecorators[i]);
            }
        }
    }
}

RealTimeAnalysisLegacy::FilteredData RealTimeAnalysisLegacy::filterData() {
    // get previous M data
    int M = 2 * parameters.filterOrder;
    auto unfilteredData = buffer.get(M, true);

    // construct the matrix that will hold the data for filtering
    Matrix rawData(M, unfilteredData[0].size());
    for (int i = 0; i < M; ++i) {
        rawData[i] = unfilteredData[i].toVector().getAsRowVector();
    }
    auto t = rawData(0);
    double dt = t[M - 1] - t[M - 2]; // assumes constant sampling rate

    // filter data and populate output struct
    int nq = unfilteredData[0].q.size();
    FilteredData filteredData;
    int delay = parameters.samplesDelay; // larger delay gives better results
    filteredData.t = t[M - delay];
    filteredData.q = Vector(nq);
    filteredData.qd = Vector(nq);
    filteredData.qdd = Vector(nq);
    int wrenchCounter = 0;
    Vector wrenchData(ExternalWrench::Input::size());
    for (int j = 1; j < rawData.ncol(); ++j) { // first column is time
        double* y = new double[M];
        Signal::LowpassFIR(parameters.filterOrder, dt, parameters.fc, M,
                           &rawData(j)[0], &y[0]);
        if (j < 1 + nq) { // first nq columns are the kinematics
            GCVSpline spline(3, M, &t[0], &y[0]);
            filteredData.q[j - 1] = spline.calcValue(Vector(1, filteredData.t));
            filteredData.qd[j - 1] =
                    spline.calcDerivative({0}, Vector(1, filteredData.t));
            filteredData.qdd[j - 1] =
                    spline.calcDerivative({0, 0}, Vector(1, filteredData.t));
        } else { // read wrench data (N = 9)
            wrenchData[wrenchCounter++] = y[M - delay];
            if (wrenchCounter == ExternalWrench::Input::size()) {
                wrenchCounter = 0;
                ExternalWrench::Input wrench;
                wrench.fromVector(wrenchData);
                filteredData.externalWrenches.push_back(wrench);
            }
        }
        delete y;
    }

    // log filtered kinematics
    // qFiltered.appendRow(filteredData.t, ~filteredData.q);
    // qDotFiltered.appendRow(filteredData.t, ~filteredData.qd);
    // qDDotFiltered.appendRow(filteredData.t, ~filteredData.qdd);

    return filteredData;
}

void RealTimeAnalysisLegacy::exportResults(string dir) {
    // inverseKinematics->logger->exportToFile(dir + "q_unfiltered.csv");
    // qFiltered->exportToFile(dir + "q_filtered.csv");
    // qDotFiltered->exportToFile(dir + "qDot_filtered.csv");
    // qDDotFiltered->exportToFile(dir + "qDDot_filtered.csv");
    // inverseDynamics->logger->exportToFile(dir + "tau.csv");
    // for (int i = 0; i < inverseDynamics->externalWrenches.size(); ++i) {
    //     inverseDynamics->externalWrenches[i]->logger->exportToFile(
    //             dir + "wrench_" + toString(i) + ".csv");
    // }
    // muscleOptimization->logger->exportToFile(dir + "so.csv");
    // jointReaction->logger->exportToFile(dir + "jr.csv");
}
