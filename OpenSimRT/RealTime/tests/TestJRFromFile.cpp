/**
 * @file TestJRFromFile.cpp
 *
 * \brief Loads results from OpenSim IK, externally applied forces and muscle
 * forces, and executes the joint reaction analysis in an iterative manner in
 * order to determine the joint reaction loads.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#include <iostream>
#include <thread>
#include <OpenSim/Simulation/Model/Muscle.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "Simulation.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

// test the alternative smoothing filter and differentiation scheme
#define IIR_FILTER

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto subjectDir = DATA_DIR + ini.Get("TESTS", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.Get("TESTS", "MODEL_FILE", "");
    auto grfMotFile = subjectDir + ini.Get("TESTS", "GRF_MOT_FILE", "");
    auto ikFile = subjectDir + ini.Get("TESTS", "IK_FILE", "");
    auto soFile = subjectDir + ini.Get("TESTS", "SO_FILE", "");

    auto grfRightApplyBody = ini.Get("TESTS", "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed = ini.Get("TESTS", "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed = ini.Get("TESTS", "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");
    auto grfRightPointIdentifier = ini.Get("TESTS", "GRF_RIGHT_POINT_IDENTIFIER", "");
    auto grfRightForceIdentifier = ini.Get("TESTS", "GRF_RIGHT_FORCE_IDENTIFIER", "");
    auto grfRightTorqueIdentifier = ini.Get("TESTS", "GRF_RIGHT_TORQUE_IDENTIFIER", "");

    auto grfLeftApplyBody = ini.Get("TESTS", "GRF_LEFT_APPLY_TO_BODY", "");
    auto grfLeftForceExpressed = ini.Get("TESTS", "GRF_LEFT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfLeftPointExpressed = ini.Get("TESTS", "GRF_LEFT_POINT_EXPRESSED_IN_BODY", "");
    auto grfLeftPointIdentifier = ini.Get("TESTS", "GRF_LEFT_POINT_IDENTIFIER", "");
    auto grfLeftForceIdentifier = ini.Get("TESTS", "GRF_LEFT_FORCE_IDENTIFIER", "");
    auto grfLeftTorqueIdentifier = ini.Get("TESTS", "GRF_LEFT_TORQUE_IDENTIFIER", "");

    Model model(modelFile);

    // read external forces
    Storage grfMotion(grfMotFile);
    ExternalWrench::Parameters grfRightFootPar{grfRightApplyBody,
					       grfRightForceExpressed,
					       grfRightPointExpressed};
    auto grfRightLabels = createGRFLabelsFromIdentifiers(grfRightPointIdentifier,
							 grfRightForceIdentifier,
							 grfRightTorqueIdentifier);
    ExternalWrench::Parameters grfLeftFootPar{grfLeftApplyBody,
					      grfLeftForceExpressed,
					      grfLeftPointExpressed};
    auto grfLeftLabels = createGRFLabelsFromIdentifiers(grfLeftPointIdentifier,
							grfLeftForceIdentifier,
							grfLeftTorqueIdentifier);
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    wrenchParameters.push_back(grfLeftFootPar);

    // prepare results from inverse kinematics
    Storage ikQ(ikFile);
    ikQ.resampleLinear(0.01);
    model.getSimbodyEngine().convertDegreesToRadians(ikQ);

    // read muscle forces
    Storage soFm(soFile);
    soFm.resampleLinear(0.01);

    if (soFm.getSize() != ikQ.getSize()) {
        THROW_EXCEPTION("ik and so storages of different size " +
                        toString(ikQ.getSize()) + " != " +
                        toString(soFm.getSize()));
    }

    // filters and differentiator
#ifdef IIR_FILTER
    IIRFilter ikFilter(model.getNumCoordinates(),
                       Vector(Vec3(1., -1.1429805, 0.4128016)),
                       Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                       IIRFilter::Signal);
    // SavitzkyGolay ikFilter(model.getNumCoordinates(), 7);
    NumericalDifferentiator dq(model.getNumCoordinates(), 2);
    NumericalDifferentiator ddq(model.getNumCoordinates(), 2);
    IIRFilter grfRightFilter(9,
                             Vector(Vec3(1., -1.1429805, 0.4128016)),
                             Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                             IIRFilter::Signal);
    IIRFilter grfLeftFilter(9,
                            Vector(Vec3(1., -1.1429805, 0.4128016)),
                            Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                            IIRFilter::Signal);

#else
    StateSpaceFilter ikFilter(model.getNumCoordinates(), 6);
    StateSpaceFilter grfRightFilter(9, 6);
    StateSpaceFilter grfLeftFilter(9, 6);
#endif

    // initialize joint reaction
    JointReaction jr(modelFile, wrenchParameters);

    // visualizer
    BasicModelVisualizer visualizer(modelFile);
    auto rightKneeForceDecorator = new ForceDecorator(Red, 0.0005, 3);
    visualizer.visualizer->addDecorationGenerator(rightKneeForceDecorator);
   
    // loop through ik storage
    for (int i = 0; i < ikQ.getSize(); ++i) {
        // read storage entry
        auto ikStateVector = ikQ.getStateVector(i);
        double t = ikStateVector->getTime();
        auto q = Vector(ikStateVector->getSize(), &ikStateVector->getData()[0]);

        // get grf force
	auto grfRightWrench = getWrenchFromStorage(t, grfRightLabels, grfMotion);
	auto grfLeftWrench = getWrenchFromStorage(t, grfLeftLabels, grfMotion);

        // get muscle force
        auto soStateVector = soFm.getStateVector(i);
        auto temp = Vector(soStateVector->getSize(), &soStateVector->getData()[0]);
        auto fm = temp(0, model.getMuscles().getSize()); // extract only muscle forces

        // filter and differentiate results
#ifdef IIR_FILTER
	// filter kinematics
        q = ikFilter.filter(q);
        auto qDot = dq.diff(t, q);
        // filter external loads
        grfRightWrench.fromVector(grfRightFilter.filter(grfRightWrench.toVector()));
        grfLeftWrench.fromVector(grfLeftFilter.filter(grfLeftWrench.toVector()));
#else
	// filter kinematics
        auto filterState = ikFilter.filter(t, q);
        q = filterState.x;
        auto qDot = filterState.xDot;
        // filter external loads
        grfRightWrench.fromVector(grfRightFilter.filter(t, grfRightWrench.toVector()).x);
        grfLeftWrench.fromVector(grfLeftFilter.filter(t, grfLeftWrench.toVector()).x);
#endif

        // execute jr
        auto jrOutput = jr.solve({t, q, qDot, fm,
				  vector<ExternalWrench::Input>{grfRightWrench,
								grfLeftWrench}});	
	// visualizer
	visualizer.update(q);
	auto kneeForce = -jrOutput.reactionWrench[2](1); // tibia_r -> 3 (pelvis, femur_r, tibia_3)
	Vec3 kneeJoint;
	visualizer.model.getSimbodyEngine()
	    .transformPosition(visualizer.state,
			       visualizer.model.getBodySet().get("tibia_r"),
			       Vec3(0),
			       visualizer.model.getGround(),
			       kneeJoint);
	rightKneeForceDecorator->update(kneeJoint, kneeForce);

	this_thread::sleep_for(chrono::milliseconds(10));
    }

    // store results
    jr.logger->exportToFile(subjectDir + "results_rt/jr.csv");
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
