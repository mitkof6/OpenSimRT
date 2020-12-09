/**
 * @file TestJRFromFile.cpp
 *
 * \brief Loads results from OpenSim IK, externally applied forces and muscle
 * forces, and executes the joint reaction analysis in an iterative manner in
 * order to determine the joint reaction loads.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <iostream>
#include <thread>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include "Simulation.h"
#include "INIReader.h"
#include "Settings.h"
#include "Utils.h"
#include "Exception.h"
#include "SignalProcessing.h"
#include "Visualization.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_JR_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.getString(section, "MODEL_FILE", "");
    auto grfMotFile = subjectDir + ini.getString(section, "GRF_MOT_FILE", "");
    auto ikFile = subjectDir + ini.getString(section, "IK_FILE", "");
    auto soFile = subjectDir + ini.getString(section, "SO_FILE", "");

    auto grfRightApplyBody = ini.getString(section, "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed = ini.getString(section, "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed = ini.getString(section, "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier = ini.getString(section, "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier = ini.getString(section, "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier = ini.getString(section, "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody = ini.getString(section, "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed = ini.getString(section, "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed = ini.getString(section, "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier = ini.getString(section, "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier = ini.getString(section, "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier = ini.getString(section, "GRF_LEFT_TORQUE_IDENTIFIER", "");

    auto memory = ini.getInteger(section, "MEMORY", 0);
    auto cutoffFreq = ini.getReal(section, "CUTOFF_FREQ", 0);
    auto delay = ini.getInteger(section, "DELAY", 0);
    auto splineOrder = ini.getInteger(section, "SPLINE_ORDER", 0);

    // setup model
    Model model(modelFile);
    auto state = model.initSystem();

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

    // get kinematics as a table with ordered coordinates
    auto qTable = OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
            model, ikFile, 0.01);

    // read muscle forces
    Storage soFm(soFile);
    soFm.resampleLinear(0.01);

    if (soFm.getSize() != qTable.getNumRows()) {
        THROW_EXCEPTION("ik and so storages of different size " +
                        toString(qTable.getNumRows()) + " != " +
                        toString(soFm.getSize()));
    }

    // setup filters
    LowPassSmoothFilter::Parameters ikFilterParam;
    ikFilterParam.numSignals = model.getNumCoordinates();
    ikFilterParam.memory = memory;
    ikFilterParam.delay = delay;
    ikFilterParam.cutoffFrequency = cutoffFreq;
    ikFilterParam.splineOrder = splineOrder;
    ikFilterParam.calculateDerivatives = true;
    LowPassSmoothFilter ikFilter(ikFilterParam);

    LowPassSmoothFilter::Parameters grfFilterParam;
    grfFilterParam.numSignals = 9;
    grfFilterParam.memory = memory;
    grfFilterParam.delay = delay;
    grfFilterParam.cutoffFrequency = cutoffFreq;
    grfFilterParam.splineOrder = splineOrder;
    grfFilterParam.calculateDerivatives = false;
    LowPassSmoothFilter grfRightFilter(grfFilterParam),
            grfLeftFilter(grfFilterParam);

    // test with state space filter
    // StateSpaceFilter ikFilter({model.getNumCoordinates(), cutoffFreq});
    // StateSpaceFilter grfRightFilter({9, cutoffFreq}), grfLeftFilter({9, cutoffFreq});

    // initialize joint reaction
    JointReaction jr(model, wrenchParameters);
    auto jrLogger = jr.initializeLogger();

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.addDecorationGenerator(leftGRFDecorator);
    auto rightKneeForceDecorator = new ForceDecorator(Red, 0.0005, 3);
    visualizer.addDecorationGenerator(rightKneeForceDecorator);

    // mean delay
    int sumDelayMS = 0;

    // loop through kinematic frames
    for (int i = 0; i < qTable.getNumRows(); ++i) {
        // get raw pose from table
        double t = qTable.getIndependentColumn()[i];
        auto qRaw = qTable.getRowAtIndex(i).getAsVector();

        // get grf forces
        auto grfRightWrench = ExternalWrench::getWrenchFromStorage(
                t, grfRightLabels, grfMotion);
        auto grfLeftWrench = ExternalWrench::getWrenchFromStorage(
                t, grfLeftLabels, grfMotion);

        // get muscle force
        auto soStateVector = soFm.getStateVector(i);
        auto temp = Vector(soStateVector->getSize(), &soStateVector->getData()[0]);
        auto fm = temp(0, model.getMuscles().getSize()); // extract only muscle forces

        // filter
        auto ikFiltered = ikFilter.filter({t, qRaw});
        auto q = ikFiltered.x;
        auto qDot = ikFiltered.xDot;
        auto qDDot = ikFiltered.xDDot;

        auto grfRightFiltered =
                grfRightFilter.filter({t, grfRightWrench.toVector()});
        grfRightWrench.fromVector(grfRightFiltered.x);
        auto grfLeftFiltered =
                grfLeftFilter.filter({t, grfLeftWrench.toVector()});
        grfLeftWrench.fromVector(grfLeftFiltered.x);

        if (!ikFiltered.isValid || !grfRightFiltered.isValid ||
            !grfLeftFiltered.isValid) {
            continue;
        }

        // perform jr
        chrono::high_resolution_clock::time_point t1;
        t1 = chrono::high_resolution_clock::now();

        auto jrOutput = jr.solve(
            {t, q, qDot, fm,
             vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

        chrono::high_resolution_clock::time_point t2;
        t2 = chrono::high_resolution_clock::now();
        sumDelayMS +=
                chrono::duration_cast<chrono::milliseconds>(t2 - t1).count();

        // visualizer
        visualizer.update(q);
        rightGRFDecorator->update(grfRightWrench.point, grfRightWrench.force);
        leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);
        // tibia_r -> 3 (pelvis, femur_r, tibia_r) // should check order
        auto kneeForce = -jrOutput.reactionWrench[2](1);
        Vec3 kneeJoint;
        state.updQ() = q;
        model.realizePosition(state);
        model.getSimbodyEngine()
            .transformPosition(state,
                               model.getBodySet().get("tibia_r"),
                               Vec3(0),
                               model.getGround(),
                               kneeJoint);
        rightKneeForceDecorator->update(kneeJoint, kneeForce);

        // log data
        jrLogger.appendRow(t, ~jr.convertOutput(jrOutput));

        // this_thread::sleep_for(chrono::milliseconds(10));
    }

    cout << "Mean delay: " << (double) sumDelayMS / qTable.getNumRows() << " ms"
         << endl;

    // store results
    STOFileAdapter::write(jrLogger,
                          subjectDir + "real_time/joint_reaction_analysis/jr.sto");
}

int main(int argc, char *argv[]) {
    try {
        run();
    } catch (exception &e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
