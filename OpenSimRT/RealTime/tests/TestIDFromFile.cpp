/**
 * @file TestIDFromFile.cpp
 *
 * \brief Loads results from OpenSim IK and externally applied forces and
 * executes the inverse dynamics analysis in an iterative manner in order to
 * determine the generalized forces.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#include "INIReader.h"
#include "OpenSimUtils.h"
#include "Settings.h"
#include "Simulation.h"
#include "Utils.h"
#include "SignalProcessing.h"
#include "Visualization.h"

#include <iostream>
#include <thread>

using namespace std;
using namespace OpenSim;
using namespace SimTK;
using namespace OpenSimRT;

// test the alternative smoothing filter and differentiation scheme
#define IIR_FILTER

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto subjectDir = DATA_DIR + ini.getString("TESTS", "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString("TESTS", "MODEL_FILE", "");
    auto grfMotFile = subjectDir + ini.getString("TESTS", "GRF_MOT_FILE", "");
    auto ikFile = subjectDir + ini.getString("TESTS", "IK_FILE", "");

    auto grfRightApplyBody =
            ini.getString("TESTS", "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed =
            ini.getString("TESTS", "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed =
            ini.getString("TESTS", "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier =
            ini.getString("TESTS", "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier =
            ini.getString("TESTS", "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier =
            ini.getString("TESTS", "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody =
            ini.getString("TESTS", "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed =
            ini.getString("TESTS", "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed =
            ini.getString("TESTS", "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier =
            ini.getString("TESTS", "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier =
            ini.getString("TESTS", "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier =
            ini.getString("TESTS", "GRF_LEFT_TORQUE_IDENTIFIER", "");

    // setup model
    Model model(modelFile);
    // ModelUtils::removeActuators(model);
    // model.finalizeConnections();

    // read external forces
    Storage grfMotion(grfMotFile);
    ExternalWrench::Parameters grfRightFootPar{
            grfRightApplyBody, grfRightForceExpressed, grfRightPointExpressed};
    auto grfRightLabels = InverseDynamics::createGRFLabelsFromIdentifiers(
            grfRightPointIdentifier, grfRightForceIdentifier,
            grfRightTorqueIdentifier);
    ExternalWrench::Parameters grfLeftFootPar{
            grfLeftApplyBody, grfLeftForceExpressed, grfLeftPointExpressed};
    auto grfLeftLabels = InverseDynamics::createGRFLabelsFromIdentifiers(
            grfLeftPointIdentifier, grfLeftForceIdentifier,
            grfLeftTorqueIdentifier);

    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // prepare results from inverse kinematics
    Storage ikQ(ikFile);
    ikQ.resampleLinear(0.01);
    // model.getSimbodyEngine().convertDegreesToRadians(ikQ);

    // filters and differentiator
#ifdef IIR_FILTER
    IIRFilter ikFilter(model.getNumCoordinates(),
                       Vector(Vec3(1., -1.1429805, 0.4128016)),
                       Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                       IIRFilter::Signal);
    // SavitzkyGolay ikFilter(model.getNumCoordinates(), 7);
    NumericalDifferentiator dq(model.getNumCoordinates(), 2);
    NumericalDifferentiator ddq(model.getNumCoordinates(), 2);
    IIRFilter grfRightFilter(9, Vector(Vec3(1., -1.1429805, 0.4128016)),
                             Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                             IIRFilter::Signal);
    IIRFilter grfLeftFilter(9, Vector(Vec3(1., -1.1429805, 0.4128016)),
                            Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                            IIRFilter::Signal);

#else
    StateSpaceFilter ikFilter(model.getNumCoordinates(), 6);
    StateSpaceFilter grfRightFilter(9, 6);
    StateSpaceFilter grfLeftFilter(9, 6);
#endif

    // initialize id
    InverseDynamics id(model, wrenchParameters);

    // visualizer
    BasicModelVisualizer visualizer(model);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.getVisualizer()->addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.getVisualizer()->addDecorationGenerator(leftGRFDecorator);

    // loop through ik storage
    for (int i = 0; i < ikQ.getSize(); ++i) {
        // read storage entry
        auto stateVector = ikQ.getStateVector(i);
        double t = stateVector->getTime();
        auto q = Vector(stateVector->getSize(), &stateVector->getData()[0]);

        // get grf force
        auto grfRightWrench = InverseDynamics::getWrenchFromStorage(
                t, grfRightLabels, grfMotion);
        auto grfLeftWrench = InverseDynamics::getWrenchFromStorage(
                t, grfLeftLabels, grfMotion);

        // filter and differentiate results
#ifdef IIR_FILTER
        // filter kinematics
        q = ikFilter.filter(q);
        auto qDot = dq.diff(t, q);
        auto qDDot = ddq.diff(t, qDot);
        // filter external loads
        grfRightWrench.fromVector(
                grfRightFilter.filter(grfRightWrench.toVector()));
        grfLeftWrench.fromVector(
                grfLeftFilter.filter(grfLeftWrench.toVector()));
#else
        // filter kinematics
        auto filterState = ikFilter.filter(t, q);
        q = filterState.x;
        auto qDot = filterState.xDot;
        auto qDDot = filterState.xDDot;
        // filter external loads
        grfRightWrench.fromVector(
                grfRightFilter.filter(t, grfRightWrench.toVector()).x);
        grfLeftWrench.fromVector(
                grfLeftFilter.filter(t, grfLeftWrench.toVector()).x);
#endif

        // execute id
        auto idOutput = id.solve(
                {t, q, qDot, qDDot,
                 vector<ExternalWrench::Input>{grfRightWrench, grfLeftWrench}});

        // visualization
        visualizer.update(q);
        rightGRFDecorator->update(grfRightWrench.point, grfRightWrench.force);
        leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);

        this_thread::sleep_for(chrono::milliseconds(10));
    }

    // store results
    // id.logger->exportToFile(subjectDir + "results_rt/tau.csv");
    // for (int i = 0; i < id.externalWrenches.size(); ++i) {
    //     id.externalWrenches[i]->logger->exportToFile(
    //             subjectDir + "results_rt/wrench_" + toString(i) + ".csv");
    // }
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
