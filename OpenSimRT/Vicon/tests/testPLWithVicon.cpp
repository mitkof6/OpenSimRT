/**
 * @file TestPLWithVicon.cpp
 *
 * \brief Interface OActive pipeline with Vicon server.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#include <iostream>
#include <OpenSim/Simulation/Model/BodySet.h>
#include "ViconDataStream.h"
#include "Simulation.h"
#include "INIReader.h"
#include "Settings.h"

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto hostName = ini.Get("VICON", "HOST_NAME", "");
    auto forcePlate00X = ini.GetReal("VICON", "FORCE_PLATE_00_X", 0);
    auto forcePlate00Y = ini.GetReal("VICON", "FORCE_PLATE_00_Y", 0);
    auto forcePlate00Z = ini.GetReal("VICON", "FORCE_PLATE_00_Z", 0);
    auto referenceFrameX = ini.Get("VICON", "REFERENCE_FRAME_AXIS_X", "");
    auto referenceFrameY = ini.Get("VICON", "REFERENCE_FRAME_AXIS_Y", "");
    auto referenceFrameZ = ini.Get("VICON", "REFERENCE_FRAME_AXIS_Z", "");
    auto subjectDir = DATA_DIR + ini.Get("VICON", "SUBJECT_DIR", "");
    auto modelFile = subjectDir +  ini.Get("VICON", "MODEL_FILE", "");
    auto grfRightApplyBody = ini.Get("VICON", "GRF_RIGHT_APPLY_TO_BODY", "");
    auto grfRightForceExpressed = ini.Get("VICON", "GRF_RIGHT_FORCE_EXPRESSED_IN_BODY", "");
    auto grfRightPointExpressed = ini.Get("VICON", "GRF_RIGHT_POINT_EXPRESSED_IN_BODY", "");

    // setup vicon
    ViconDataStream vicon(vector<Vec3>{Vec3(forcePlate00X,
					    forcePlate00Y,
					    forcePlate00Z)});
    vicon.connect(hostName);
    vicon.initialize(stringToDirection(referenceFrameX),
		     stringToDirection(referenceFrameY),
		     stringToDirection(referenceFrameZ));

    // prepare marker tasks and ik
    Model model(modelFile);
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    createMarkerTasksFromMarkerNames(model, vicon.markerNames,
				     markerTasks, observationOrder);
    InverseKinematics ik(modelFile, 100, markerTasks,
                         vector<InverseKinematics::IMUTask>{});

    // ground reaction forces and id
    ExternalWrench::Parameters grfRightFootPar{grfRightApplyBody,
					       grfRightForceExpressed,
					       grfRightPointExpressed};
    vector<ExternalWrench::Parameters> wrenchParameters;
    wrenchParameters.push_back(grfRightFootPar);
    auto forcePlateNames = vicon.forcePlateNames;
    InverseDynamics id(modelFile, wrenchParameters);

    // initialize so
    MuscleOptimization so(modelFile,
			  MuscleOptimization::OptimizationParameters(),
			  momentArmSelector(modelFile),
              new TorqueBasedTargetNonLinearMuscle());

    // initialize joint reaction
    JointReaction jr(modelFile, wrenchParameters);

    // filters
    IIRFilter ikFilter(model.getNumCoordinates(),
                       Vector(Vec3(1., -1.1429805, 0.4128016)),
                       Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                       IIRFilter::Signal);
    NumericalDifferentiator dq(model.getNumCoordinates(), 2);
    NumericalDifferentiator ddq(model.getNumCoordinates(), 2);
    IIRFilter grfRightFilter(9,
                             Vector(Vec3(1., -1.1429805, 0.4128016)),
                             Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                             IIRFilter::Signal);

    // initialize loggers
    auto coordinateColumnNames = getCoordinateNames(model);
    coordinateColumnNames.insert(coordinateColumnNames.begin(), "time");
    CSVLogger qFiltered(coordinateColumnNames),
	qDotFiltered(coordinateColumnNames),
	qDDotFiltered(coordinateColumnNames);

    // visualizer
    BasicModelVisualizer visualizer(modelFile);
    auto rightGRFDecorator = new ForceDecorator(Green, 0.001, 3);
    visualizer.visualizer->addDecorationGenerator(rightGRFDecorator);
    auto rightKneeForceDecorator = new ForceDecorator(Red, 0.0005, 3);
    visualizer.visualizer->addDecorationGenerator(rightKneeForceDecorator);

    // main loop
    vicon.startAcquisition();
    double previousTime= -1.0;
    while(!visualizer.shouldTerminate) {
	auto markerData = vicon.markerBuffer.get(1)[0];
	auto forceData = vicon.forceBuffer.get(10, true)[0];
	double t = markerData.time;
	if (previousTime >= t) {
    	    continue;
    	}

        // ik
        InverseKinematics::Input ikInput;
	ikInput.t = t;
        for (auto markerName : observationOrder) {
            ikInput.markerObservations.push_back(markerData.markers[markerName]);
        }
        auto ikOutput = ik.solve(ikInput);
	auto q = ikOutput.q;

	// filter kinematics
        q = ikFilter.filter(q);
        auto qDot = dq.diff(t, q);
        auto qDDot = ddq.diff(t, qDot);

        // filter external loads
	auto grfRightWrench = forceData.externalWrenches[forcePlateNames[0]];
        grfRightWrench.fromVector(grfRightFilter.filter(grfRightWrench.toVector()));
	auto externalWrenches = vector<ExternalWrench::Input>{grfRightWrench};

        // solve id
        auto idOutput = id.solve({t, q, qDot, qDDot, externalWrenches});
	auto tau = idOutput.tau;

	// solve so
        auto soOutput = so.solve({t, q, tau});
        auto fm = soOutput.fm;
	auto am = soOutput.am;

        // solve jr
        auto jrOutput = jr.solve({t, q, qDot, fm, externalWrenches});

        // visualizer
        visualizer.update(q, am);
	rightGRFDecorator->update(grfRightWrench.point, grfRightWrench.force);
	auto kneeForce = -jrOutput.reactionWrench[2](1); // tibia_r -> 3 (pelvis, femur_r, tibia_3)
	Vec3 kneeJoint;
	visualizer.model.getSimbodyEngine()
	    .transformPosition(visualizer.state,
			       visualizer.model.getBodySet().get("tibia_r"),
			       Vec3(0),
			       visualizer.model.getGround(),
			       kneeJoint);
	rightKneeForceDecorator->update(kneeJoint, kneeForce);

	previousTime = t;

	// log filter results
        qFiltered.addRow(t, q);
        qDotFiltered.addRow(t, qDot);
        qDDotFiltered.addRow(t, qDDot);
    }

    // store results
    ik.logger->exportToFile(subjectDir + "results_rt/q_unfiltered.csv");
    qFiltered.exportToFile(subjectDir + "results_rt/q_filtered.csv");
    qDotFiltered.exportToFile(subjectDir + "results_rt/qDot_filtered.csv");
    qDDotFiltered.exportToFile(subjectDir + "results_rt/qDDot_filtered.csv");
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
