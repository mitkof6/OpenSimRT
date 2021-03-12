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
 *
 * @file TestRTExtFromFile.cpp
 *
 * @brief Tests the RealTimeAnalysisExtended class with data acquired from file.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "ContactForceBasedPhaseDetector.h"
#include "INIReader.h"
#include "JointReaction.h"
#include "RealTimeAnalysisExtended.h"
#include "Settings.h"
#include "Visualization.h"

#include <Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <iostream>
#include <thread>

using namespace std;
using namespace SimTK;
using namespace OpenSim;
using namespace OpenSimRT;

void run(char const* name) {
    INIReader ini(INI_FILE);
    auto section = "TEST_RT_EXTENDED_PIPELINE_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto trcFile = subjectDir + ini.getString(section, "TRC_FILE", "");
    auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");

    // grf body and identifier labels
    auto grfRightApplyBody =
            ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed =
            ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed =
            ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier =
            ini.getString(section, "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier =
            ini.getString(section, "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier =
            ini.getString(section, "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody =
            ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed =
            ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed =
            ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier =
            ini.getString(section, "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier =
            ini.getString(section, "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier =
            ini.getString(section, "GRF_LEFT_TORQUE_IDENTIFIER", "");

    // Low pass filter parameters
    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);
    auto calcDer = ini.getBoolean(section, "CALC_DER", true);

    // contact-force-based detector parameters
    auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
    auto threshold = ini.getReal(section, "THRESHOLD", 0);
    auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);
    auto rFootBodyName = ini.getString(section, "RIGHT_FOOT_BODY_NAME", "");
    auto lFootBodyName = ini.getString(section, "LEFT_FOOT_BODY_NAME", "");
    auto rHeelSphereLocation =
            ini.getSimtkVec(section, "RIGHT_HEEL_SPHERE_LOCATION", Vec3(0));
    auto lHeelSphereLocation =
            ini.getSimtkVec(section, "LEFT_HEEL_SPHERE_LOCATION", Vec3(0));
    auto rToeSphereLocation =
            ini.getSimtkVec(section, "RIGHT_TOE_SPHERE_LOCATION", Vec3(0));
    auto lToeSphereLocation =
            ini.getSimtkVec(section, "LEFT_TOE_SPHERE_LOCATION", Vec3(0));
    auto contactSphereRadius = ini.getReal(section, "SPHERE_RADIUS", 0);

    // grfm parameters
    bool useGRFMPrediction =
            ini.getBoolean(section, "USE_GRFM_PREDICTION", true);
    auto grfmMethod = ini.getString(section, "METHOD", "");
    auto pelvisBodyName = ini.getString(section, "PELVIS_BODY_NAME", "");
    auto rHeelCoPLocation =
            ini.getSimtkVec(section, "RIGHT_HEEL_STATION_LOCATION", Vec3(0));
    auto lHeelCoPLocation =
            ini.getSimtkVec(section, "LEFT_HEEL_STATION_LOCATION", Vec3(0));
    auto rToeCoPLocation =
            ini.getSimtkVec(section, "RIGHT_TOE_STATION_LOCATION", Vec3(0));
    auto lToeCoPLocation =
            ini.getSimtkVec(section, "LEFT_TOE_STATION_LOCATION", Vec3(0));
    auto directionWindowSize =
            ini.getInteger(section, "DIRECTION_WINDOW_SIZE", 0);

    // so parameters
    bool solveMuscleOptimization = ini.getBoolean(section, "SOLVE_SO", true);
    auto convergenceTolerance =
            ini.getReal(section, "CONVERGENCE_TOLERANCE", 0.0);
    auto memoryHistory = ini.getInteger(section, "MEMORY_HISTORY", 0);
    auto maximumIterations = ini.getInteger(section, "MAXIMUM_ITERATIONS", 0);
    auto objectiveExponent = ini.getInteger(section, "OBJECTIVE_EXPONENT", 0);
    auto momentArmLibraryPath =
            ini.getString(section, "MOMENT_ARM_LIBRARY", "");

    // prepare model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    auto state = model.initSystem();
    const int dofs = state.getNU();
    const int joints = model.getJointSet().getSize();
    const int muscles = model.getMuscles().getSize();

    // load moment arm function
    auto calcMomentArm = OpenSimUtils::getMomentArmFromDynamicLibrary(
            model, momentArmLibraryPath);

    // prepare marker tasks
    MarkerData markerData(trcFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromMarkerData(
            model, markerData, markerTasks, observationOrder);

    // read external forces
    Storage grfMotion(grfMotFile);
    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    auto grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    auto grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // acquisition function (simulates acquisition from motion)
    auto dataAcquisitionFunction = [&]() -> MotionCaptureInput {
        static int i = 0;
        MotionCaptureInput input;

        // get frame data
        input.IkFrame = InverseKinematics::getFrameFromMarkerData(
                i, markerData, observationOrder, false);
        double t = input.IkFrame.t;

        // dummy delay to simulate real time
        this_thread::sleep_for(chrono::milliseconds(16));
        i++;
        return input;
    };

    // initialize filter parameters
    LowPassSmoothFilter::Parameters filterParameters;
    filterParameters.numSignals = dofs;
    filterParameters.memory = memory;
    filterParameters.delay = delay;
    filterParameters.cutoffFrequency = cutoffFreq;
    filterParameters.splineOrder = splineOrder;
    filterParameters.calculateDerivatives = calcDer;

    // contact force based event detection
    ContactForceBasedPhaseDetector::Parameters detectorParameters;
    detectorParameters.threshold = threshold;
    detectorParameters.windowSize = windowSize;
    detectorParameters.plane_origin = Vec3(0.0, platform_offset, 0.0);
    detectorParameters.rHeelSphereLocation = rHeelSphereLocation;
    detectorParameters.lHeelSphereLocation = lHeelSphereLocation;
    detectorParameters.rToeSphereLocation = rToeSphereLocation;
    detectorParameters.lToeSphereLocation = lToeSphereLocation;
    detectorParameters.sphereRadius = contactSphereRadius;
    detectorParameters.rFootBodyName = rFootBodyName;
    detectorParameters.lFootBodyName = lFootBodyName;
    auto detector = ContactForceBasedPhaseDetector(model, detectorParameters);

    // grfm prediction
    GRFMPrediction::Parameters grfmParameters;
    grfmParameters.method = GRFMPrediction::selectMethod(grfmMethod);
    grfmParameters.pelvisBodyName = pelvisBodyName;
    grfmParameters.rStationBodyName = rFootBodyName;
    grfmParameters.lStationBodyName = lFootBodyName;
    grfmParameters.rHeelStationLocation = rHeelCoPLocation;
    grfmParameters.lHeelStationLocation = lHeelCoPLocation;
    grfmParameters.rToeStationLocation = rToeCoPLocation;
    grfmParameters.lToeStationLocation = lToeCoPLocation;
    grfmParameters.directionWindowSize = directionWindowSize;

    // so parameters
    MuscleOptimization::OptimizationParameters muscleOptimizationParameters;
    muscleOptimizationParameters.convergenceTolerance = convergenceTolerance;
    muscleOptimizationParameters.memoryHistory = memoryHistory;
    muscleOptimizationParameters.maximumIterations = maximumIterations;
    muscleOptimizationParameters.objectiveExponent = objectiveExponent;

    // pipeline
    RealTimeAnalysisExtended::Parameters pipelineParameters;
    pipelineParameters.solveMuscleOptimization = solveMuscleOptimization;
    pipelineParameters.ikMarkerTasks = markerTasks;
    pipelineParameters.ikConstraintsWeight = SimTK::Infinity;
    pipelineParameters.ikAccuracy = 1e-5;
    pipelineParameters.filterParameters = filterParameters;
    pipelineParameters.muscleOptimizationParameters =
            muscleOptimizationParameters;
    pipelineParameters.wrenchParameters = wrenchParameters;
    pipelineParameters.dataAcquisitionFunction = dataAcquisitionFunction;
    pipelineParameters.momentArmFunction = calcMomentArm;
    pipelineParameters.useGRFMPrediction = useGRFMPrediction;
    pipelineParameters.phaseDetector = detector;
    pipelineParameters.detectorUpdateMethod =
            RealTimeAnalysisExtended::PhaseDetectorUpdateMethod::INTERNAL;
    pipelineParameters.grfmParameters = grfmParameters;
    pipelineParameters.internalPhaseDetectorUpdateFunction =
            [&](const double& t, const SimTK::Vector& q,
                const SimTK::Vector& qd, const SimTK::Vector& qdd) {
                detector.updDetector({t, q, qd, qdd});
            };
    RealTimeAnalysisExtended pipeline(model, pipelineParameters);
    auto log = pipeline.initializeLoggers();

    // run pipeline
    pipeline.run();

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(leftGRFDecorator);
    auto rightKneeForceDecorator = new ForceDecorator(Red, 0.0005, 3);
    visualizer.addDecorationGenerator(rightKneeForceDecorator);
    auto leftKneeForceDecorator = new ForceDecorator(Red, 0.0005, 3);
    visualizer.addDecorationGenerator(leftKneeForceDecorator);

    try {
        while (!pipeline.shouldTerminate()) {
            // fetch of rt results
            auto results = pipeline.getResults();

            // update visualizer
            if (!solveMuscleOptimization)
                visualizer.update(results.q);
            else {
                visualizer.update(results.q, results.am);
                visualizer.updateReactionForceDecorator(
                        results.reactionWrenches, "tibia_r",
                        rightKneeForceDecorator);
                visualizer.updateReactionForceDecorator(
                        results.reactionWrenches, "tibia_l",
                        leftKneeForceDecorator);
            }

            // log
            log.qLogger.appendRow(results.t, ~results.q);
            log.qDotLogger.appendRow(results.t, ~results.qd);
            log.qDDotLogger.appendRow(results.t, ~results.qdd);
            log.tauLogger.appendRow(results.t, ~results.tau);
            log.fmLogger.appendRow(results.t, ~results.fm);
            log.amLogger.appendRow(results.t, ~results.am);
            log.residualLogger.appendRow(results.t, ~results.residuals);
            log.jrLogger.appendRow(results.t, ~results.reactionWrenchVector);

        } // while loop
    } catch (const exception& e) {
        cout << e.what() << "\n";
        pipeline.shouldTerminate(true);
    }

    // // store results
    // STOFileAdapter::write(log.qLogger,
    //                       subjectDir + "real_time/pipeline/ext/q.sto");
    // STOFileAdapter::write(log.qDotLogger,
    //                       subjectDir + "real_time/pipeline/ext/qDot.sto");
    // STOFileAdapter::write(log.qDDotLogger,
    //                       subjectDir + "real_time/pipeline/ext/qDDot.sto");
    // STOFileAdapter::write(log.tauLogger,
    //                       subjectDir + "real_time/pipeline/ext/tau.sto");
    // STOFileAdapter::write(log.amLogger,
    //                       subjectDir + "real_time/pipeline/ext/am.sto");
    // STOFileAdapter::write(log.fmLogger,
    //                       subjectDir + "real_time/pipeline/ext/fm.sto");
    // STOFileAdapter::write(log.residualLogger,
    //                       subjectDir +
    //                       "real_time/pipeline/ext/residuals.sto");
    // STOFileAdapter::write(log.jrLogger,
    //                       subjectDir + "real_time/pipeline/ext/jr.sto");
}
int main(int argc, char* argv[]) {
    try {
        run(argc > 1 ? argv[1] : nullptr);
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
