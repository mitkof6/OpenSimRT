/**
 * @file TestPLFromFile.cpp
 *
 * \brief Loads the marker trajectories and executes inverse kinematics in an
 * iterative manner in order to determine the model kinematics.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#include <iostream>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "Simulation.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

// test the alternative smoothing filter and differentiation scheme
#define IIR_FILTER

// use plotting (should be disabled for testing)
// #define USE_GNUPLOT

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto subjectDir = DATA_DIR + ini.Get("TESTS", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.Get("TESTS", "MODEL_FILE", "");
    auto trcFile = subjectDir +  ini.Get("TESTS", "TRC_FILE", "");
    auto ikTaskSetFile = subjectDir +  ini.Get("TESTS", "IK_TASK_SET_FILE", "");
    auto grfMotFile = subjectDir + ini.Get("TESTS", "GRF_MOT_FILE", "");

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

    // prepare marker tasks
    IKTaskSet ikTaskSet(ikTaskSetFile);
    MarkerData markerData(trcFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    createMarkerTasksFromIKTaskSet(model, ikTaskSet,
                                   markerTasks, observationOrder);

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
    IIRFilter idFilter(model.getNumCoordinates(),
                       Vector(Vec3(1., -1.1429805, 0.4128016)),
                       Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                       IIRFilter::Signal);
#else
    StateSpaceFilter ikFilter(model.getNumCoordinates(), 6);
    StateSpaceFilter grfRightFilter(9, 6);
    StateSpaceFilter grfLeftFilter(9, 6);
    StateSpaceFilter idFilter(model.getNumCoordinates(), 6);
#endif

    // initialize loggers
    auto coordinateColumnNames = getCoordinateNames(model);
    coordinateColumnNames.insert(coordinateColumnNames.begin(), "time");
    CSVLogger qFiltered(coordinateColumnNames),
	qDotFiltered(coordinateColumnNames),
	qDDotFiltered(coordinateColumnNames);
    CSVLogger tauLogger(coordinateColumnNames);

    // initialize ik
    InverseKinematics ik(modelFile, 100, markerTasks,
			 vector<InverseKinematics::IMUTask>{});

    // initialize id
    InverseDynamics id(modelFile, wrenchParameters);

    // initialize so
    MuscleOptimization so(modelFile,
			  MuscleOptimization::OptimizationParameters(),
			  momentArmSelector(modelFile),
              new TorqueBasedTargetNonLinearMuscle());

    // initialize joint reaction
    JointReaction jr(modelFile, wrenchParameters);

    // visualizer
    BasicModelVisualizer visualizer(modelFile);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.visualizer->addDecorationGenerator(rightGRFDecorator);
    auto leftGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.visualizer->addDecorationGenerator(leftGRFDecorator);
    auto rightKneeForceDecorator = new ForceDecorator(Red, 0.0005, 3);
    visualizer.visualizer->addDecorationGenerator(rightKneeForceDecorator);

    // gnuplot
#ifdef USE_GNUPLOT && __linux__
    GNUPlot<500, 2> plot(false);
#endif
    // getchar();
    // loop through marker frames
    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        // get frame data
        auto frame = getFrameFromMarkerData(i, markerData, observationOrder, false);
        double t = frame.t;

        // get grf force
        auto grfRightWrench = getWrenchFromStorage(t, grfRightLabels, grfMotion);
        auto grfLeftWrench = getWrenchFromStorage(t, grfLeftLabels, grfMotion);

        // solve ik
        auto pose = ik.solve(frame);
        auto q = pose.q;

        // filter and differentiate results
#ifdef IIR_FILTER
	// filter kinematics
        q = ikFilter.filter(q);
        auto qDot = dq.diff(t, q);
        auto qDDot = ddq.diff(t, qDot);
        // filter external loads
        grfRightWrench.fromVector(grfRightFilter.filter(grfRightWrench.toVector()));
        grfLeftWrench.fromVector(grfLeftFilter.filter(grfLeftWrench.toVector()));
#else
	// filter kinematics
        auto filterState = ikFilter.filter(t, q);
        q = filterState.x;
        auto qDot = filterState.xDot;
        auto qDDot = filterState.xDDot;
        // filter external loads
        grfRightWrench.fromVector(grfRightFilter.filter(t, grfRightWrench.toVector()).x);
        grfLeftWrench.fromVector(grfLeftFilter.filter(t, grfLeftWrench.toVector()).x);
#endif

	auto externalForce = vector<ExternalWrench::Input>{grfRightWrench,
							   grfLeftWrench};

        // solve id
        auto idOutput = id.solve({t, q, qDot, qDDot, externalForce});

        // filter id
#ifdef IIR_FILTER
        auto tau = idFilter.filter(idOutput.tau);
#else
        auto tau = idFilter.filter(t, idOutput.tau).x;
#endif

        // solve so
        auto soOutput = so.solve({t, q, tau});
        auto fm = soOutput.fm;
	auto am = soOutput.am;

        // solve jr
        auto jrOutput = jr.solve({t, q, qDot, fm, externalForce});

	// visualizer
	visualizer.update(q, am);
	rightGRFDecorator->update(grfRightWrench.point, grfRightWrench.force);
	leftGRFDecorator->update(grfLeftWrench.point, grfLeftWrench.force);

	auto kneeForce = -jrOutput.reactionWrench[2](1); // tibia_r -> 3 (pelvis, femur_r, tibia_3)
	Vec3 kneeJoint;
	visualizer.model.getSimbodyEngine()
	    .transformPosition(visualizer.state,
			       visualizer.model.getBodySet().get("tibia_r"),
			       Vec3(0),
			       visualizer.model.getGround(),
			       kneeJoint);
	rightKneeForceDecorator->update(kneeJoint, kneeForce);

	// log filter results
        qFiltered.addRow(t, q);
        qDotFiltered.addRow(t, qDot);
        qDDotFiltered.addRow(t, qDDot);
	tauLogger.addRow(idOutput.t, tau);

#ifdef USE_GNUPLOT && __linux__
        // visualize
        Vector temp(2);
        temp[0] = t;
        temp[1] = jrOutput.reactionWrench[2](1)[1];  // tibia body, force, y
        plot.add(temp);
        plot.visualize({1}, {"lp"}, {"knee fy"});
#endif
    }

    // store results
    ik.logger->exportToFile(subjectDir + "results_rt/q_unfiltered.csv");
    qFiltered.exportToFile(subjectDir + "results_rt/q_filtered.csv");
    qDotFiltered.exportToFile(subjectDir + "results_rt/qDot_filtered.csv");
    qDDotFiltered.exportToFile(subjectDir + "results_rt/qDDot_filtered.csv");
    tauLogger.exportToFile(subjectDir + "results_rt/tau.csv");
    for (int i = 0; i < id.externalWrenches.size(); ++i) {
	id.externalWrenches[i]->logger->exportToFile(
	    subjectDir + "results_rt/wrench_" + toString(i) + ".csv");
    }
    so.logger->exportToFile(subjectDir + "results_rt/so.csv");
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
