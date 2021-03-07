/**
 * @file TestContactForceBasedGRFMPredictionFromFile.cpp
 *
 * @brief Test the GRF&M prediction method with the ContactForceBased
 * PhaseDetector. Increases the simulation time by repeating the recorded motion
 * X times, in order to provide enough time for the detector to adapt.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "ContactForceBasedPhaseDetector.h"
#include "GRFMPrediction.h"
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "SignalProcessing.h"
#include "Simulation.h"
#include "Utils.h"
#include "Visualization.h"

#include <Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/STOFileAdapter.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_CONTACT_FORCE_GRFM_PREDICTION_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");

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
    auto grfOrigin = ini.getSimtkVec(section, "GRF_ORIGIN", Vec3(0));

    // virtual contact surface as ground
    auto platform_offset = ini.getReal(section, "PLATFORM_OFFSET", 0.0);

    // repeat cyclic motion X times
    auto simulationLoops = ini.getInteger(section, "SIMULATION_LOOPS", 0);
    // remove last N samples in motion for smooth transition between loops
    auto removeNLastRows =
            ini.getInteger(section, "REMOVE_N_LAST_TABLE_ROWS", 0);

    // filter
    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // contact-force-based detector parameters
    auto windowSize = ini.getInteger(section, "WINDOW_SIZE", 0);
    auto threshold = ini.getReal(section, "THRESHOLD", 0);
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

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);
    model.initSystem();

    // setup external forces parameters
    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    auto grfRightLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    auto grfRightLogger = ExternalWrench::initializeLogger();

    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    auto grfLeftLabels = ExternalWrench::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);
    auto grfLeftLogger = ExternalWrench::initializeLogger();

    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    // remove last rows in qTable
    for (int i = 0; i < removeNLastRows; ++i)
        qTable.removeRow(qTable.getIndependentColumn().back());

    // setup filters
    LowPassSmoothFilter::Parameters filterParam;
    filterParam.numSignals = model.getNumCoordinates();
    filterParam.memory = memory;
    filterParam.delay = delay;
    filterParam.cutoffFrequency = cutoffFreq;
    filterParam.splineOrder = splineOrder;
    filterParam.calculateDerivatives = true;
    LowPassSmoothFilter filter(filterParam);

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
    GRFMPrediction grfm(model, grfmParameters, &detector);

    // id
    InverseDynamics id(model, wrenchParameters);
    auto tauLogger = id.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(leftGRFDecorator);

    // mean delay
    int sumDelayMS;
    int sumDelayMSCounter;

    int loopCounter = 0;
    int i = 0;
    // repeat the simulation `simulationLoops` times
    for (int k = 0; k < qTable.getNumRows() * simulationLoops; k++) {
        // get raw pose from table
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();
        double t = qTable.getIndependentColumn()[i];

        // increment the time by the total simulation time plus the sampling
        // period, to keep increasing after each simulation loop
        t += loopCounter * (qTable.getIndependentColumn().back() + 0.01);

        // filter
        auto ikFiltered = filter.filter({t, qRaw});
        auto q = ikFiltered.x;
        auto qDot = ikFiltered.xDot;
        auto qDDot = ikFiltered.xDDot;

        // increment loop
        if (++i == qTable.getNumRows()) {
            i = 0;
            loopCounter++;
        }
        if (!ikFiltered.isValid) { continue; }

        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();

        // perform grfm prediction
        detector.updDetector({ikFiltered.t, q, qDot, qDDot});
        auto grfmOutput = grfm.solve({ikFiltered.t, q, qDot, qDDot});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();
        sumDelayMSCounter++;

        // project on plane
        grfmOutput.right.point =
                projectionOnPlane(grfmOutput.right.point, grfOrigin);
        grfmOutput.left.point =
                projectionOnPlane(grfmOutput.left.point, grfOrigin);

        // setup ID inputn
        ExternalWrench::Input grfRightWrench = {grfmOutput.right.point,
                                                grfmOutput.right.force,
                                                grfmOutput.right.torque};
        ExternalWrench::Input grfLeftWrench = {grfmOutput.left.point,
                                               grfmOutput.left.force,
                                               grfmOutput.left.torque};

        // solve ID
        auto idOutput = id.solve(
                {ikFiltered.t, q, qDot, qDDot,
                 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

        // visualization
        visualizer.update(q);
        rightGRFDecorator->update(grfmOutput.right.point,
                                  grfmOutput.right.force);
        leftGRFDecorator->update(grfmOutput.left.point, grfmOutput.left.force);

        // log data (use filter time to align with delay)
        grfRightLogger.appendRow(grfmOutput.t, ~grfmOutput.right.toVector());
        grfLeftLogger.appendRow(grfmOutput.t, ~grfmOutput.left.toVector());
        tauLogger.appendRow(ikFiltered.t, ~idOutput.tau);
    }

    cout << "Mean delay: " << double(sumDelayMS) / sumDelayMSCounter << " ms"
         << endl;

    // relax tolerance because of floating point errors between target machines
    OpenSimUtils::compareTables(
            grfRightLogger,
            TimeSeriesTable(subjectDir + "real_time/grfm_prediction/"
                            "force_based/wrench_right.sto"),
            1e-1);
    OpenSimUtils::compareTables(grfLeftLogger,
                                TimeSeriesTable(subjectDir +
                                                "real_time/grfm_prediction/"
                                                "force_based/wrench_left.sto"),
                                1e-1);
    OpenSimUtils::compareTables(
            tauLogger,
            TimeSeriesTable(subjectDir +
                            "real_time/grfm_prediction/force_based/tau.sto"),
            1e-1);

    // // store results
    // STOFileAdapter::write(
    //         grfRightLogger,
    //         subjectDir +
    //                 "real_time/grfm_prediction/force_based/wrench_right.sto");
    // STOFileAdapter::write(
    //         grfLeftLogger,
    //         subjectDir +
    //                 "real_time/grfm_prediction/force_based/wrench_left.sto");
    // STOFileAdapter::write(
    //         tauLogger,
    //         subjectDir + "real_time/grfm_prediction/force_based/tau.sto");
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
