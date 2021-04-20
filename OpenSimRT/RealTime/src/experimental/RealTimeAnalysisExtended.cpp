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
#include "RealTimeAnalysisExtended.h"
#include "Exception.h"
#include "InverseDynamics.h"

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

RealTimeAnalysisExtended::RealTimeAnalysisExtended(const Model& otherModel,
                                                   const Parameters& parameters)
        : RealTimeAnalysis(otherModel,
                           *static_cast<const RealTimeAnalysis::Parameters*>(
                                   &parameters)),
          parameters(parameters) {
    // create MarkerReconstruction instance
    markerReconstruction =
            new MarkerReconstruction(model, parameters.ikMarkerTasks);

    // create optionally GRFMPrediction instance. requires a valid reference to
    // an instance of GaitPhaseDetector.
    if (parameters.useGRFMPrediction) {
        if (parameters.phaseDetector == nullptr)
            THROW_EXCEPTION("Phase detector is null");
        grfmPrediction = new GRFMPrediction(model, parameters.grfmParameters,
                                            parameters.phaseDetector.get());
    }
}

void RealTimeAnalysisExtended::acquisition() {
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

            // reconstruct possible missing markers. requires at least one valid
            // frame with all markers positions
            if (!markerReconstruction->initState(
                        acquisitionData.IkFrame.markerObservations))
                continue;
            markerReconstruction->solve(
                    acquisitionData.IkFrame.markerObservations);

            // perform ik
            auto pose = inverseKinematics->solve(acquisitionData.IkFrame);

            // filter ik results
            auto unfilteredData = prepareUnfilteredData(
                    pose.q, acquisitionData.ExternalWrenches);
            auto filteredData = lowPassFilter->filter({pose.t, unfilteredData});

            // skip if filter is not ready
            if (!filteredData.isValid) continue;

            // represent filtered data as struct
            FilteredData data;
            data.fromVector(filteredData.t, filteredData.x, filteredData.xDot,
                            filteredData.xDDot, model.getNumCoordinates());

            // grfm prediction
            if (parameters.useGRFMPrediction) {
                // update detector
                if (parameters.detectorUpdateMethod ==
                    PhaseDetectorUpdateMethod::INTERNAL)
                    parameters.internalPhaseDetectorUpdateFunction(
                            data.t, data.q, data.qd, data.qdd);
                else if (parameters.detectorUpdateMethod ==
                         PhaseDetectorUpdateMethod::EXTERNAL)
                    parameters.externalPhaseDetectorUpdateFunction();
                else
                    THROW_EXCEPTION("Wrong detector update method");

                // solve grfm prediction
                auto grfmOutput = grfmPrediction->solve(
                        {data.t, data.q, data.qd, data.qdd});

                // setup wrenches
                ExternalWrench::Input grfRightWrench = {
                        grfmOutput.right.point, grfmOutput.right.force,
                        grfmOutput.right.torque};
                ExternalWrench::Input grfLeftWrench = {grfmOutput.left.point,
                                                       grfmOutput.left.force,
                                                       grfmOutput.left.torque};

                // set external wrenches in data struct
                data.externalWrenches = {grfRightWrench, grfLeftWrench};
            }

            // push to buffer
            buffer.add(data);
        }
    } catch (exception& e) {
        cout << e.what() << endl;

        // raise termination flag
        terminationFlag = true;

        // notify buffer to not get stuck in processing thread
        buffer.setDataRetrievalMode(DataRetrievalMode::CONTINUOUS);
    }
}

void RealTimeAnalysisExtended::processing() {
    try {
        Vector am, fm, residuals, reactionWrenchVector;
        Vector_<SpatialVec> reactionWrenches;
        while (true) {
            if (shouldTerminate()) THROW_EXCEPTION("Processing terminated.");

            // get data from buffer
            auto data = buffer.get(1)[0];

            // solve id
            auto id = inverseDynamics->solve(
                    {data.t, data.q, data.qd, data.qdd, data.externalWrenches});

            // solve so and jr
            if (parameters.solveMuscleOptimization) {
                auto so = muscleOptimization->solve({data.t, data.q, id.tau});
                am = so.am;
                fm = so.fm;
                residuals = so.residuals;

                auto jr = jointReaction->solve({data.t, data.q, data.qd, so.fm,
                                                data.externalWrenches});
                reactionWrenches = jr.reactionWrench;
                reactionWrenchVector = jointReaction->asForceMomentPoint(jr);
            }

            { // thread-safe write to output
                lock_guard<mutex> locker(mu);
                output.t = data.t;
                output.q = data.q;
                output.qd = data.qd;
                output.qdd = data.qdd;
                output.grfRightWrench = data.externalWrenches[0].toVector();
                output.grfLeftWrench = data.externalWrenches[1].toVector();
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
