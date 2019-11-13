/**
 * @file testSOFromFile.cpp
 *
 * \brief Loads results from OpenSim IK and externally applied forces and
 * executes the static optimization analysis in an iterative manner in order to
 * determine the muscle forces.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 */
#include <iostream>
#include <OpenSim/Simulation/Model/Muscle.h>
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
    auto ikFile = subjectDir + ini.Get("TESTS", "IK_FILE", "");
    auto idFile = subjectDir + ini.Get("TESTS", "ID_FILE", "");

    Model model(modelFile);

    // prepare results from inverse kinematics
    Storage ikQ(ikFile);
    ikQ.resampleLinear(0.01);
    model.getSimbodyEngine().convertDegreesToRadians(ikQ);

    // read external forces
    Storage id(idFile);
    id.resampleLinear(0.01);

    if (id.getSize() != ikQ.getSize()) {
        THROW_EXCEPTION("ik and id storages of different size " +
                        toString(ikQ.getSize()) + " != " +
                        toString(id.getSize()));
    }

    // filters and differentiator
#ifdef IIR_FILTER
    IIRFilter filter(model.getNumCoordinates(),
                     Vector(Vec3(1., -1.1429805, 0.4128016)),
                     Vector(Vec3(0.06745527, 0.13491055, 0.06745527)),
                     IIRFilter::Signal);
    // SavitzkyGolay filter(model.getNumCoordinates(), 7);
#else
    StateSpaceFilter filter(model.getNumCoordinates(), 6);
#endif

    // initialize so
    MuscleOptimization::OptimizationParameters optimizationParameters;
    optimizationParameters.convergenceTolerance = 1e-6; // set 1e-0 for linear muscle
    MuscleOptimization so(modelFile,
			  optimizationParameters,
			  momentArmSelector(modelFile),
              new TorqueBasedTargetNonLinearMuscle()); // TODO change tolerance
            //   new TorqueBasedTargetLinearMuscle());

    // visualizer
    BasicModelVisualizer visualizer(modelFile);

    // loop through ik storage
    for (int i = 0; i < ikQ.getSize(); ++i) {
        // read ik entry
        auto ikStateVector = ikQ.getStateVector(i);
        double t = ikStateVector->getTime();
        auto q = Vector(ikStateVector->getSize(), &ikStateVector->getData()[0]);

        // read id entry
        auto idStateVector = id.getStateVector(i);
        auto tau = Vector(idStateVector->getSize(), &idStateVector->getData()[0]);

        // filter ik
#ifdef IIR_FILTER
        q = filter.filter(q);
#else
        auto filterState = filter.filter(t, q);
        q = filterState.x;
#endif
        // compute generalized velocities
        NumericalDifferentiator dq(model.getNumCoordinates(), 2);
        auto qdot = dq.diff(t, q);

        // execute so
        auto soOutput = so.solve({t, q, qdot, tau});

	// visualization
	visualizer.update(q, soOutput.am);
    }

    // store results
    so.logger->exportToFile(subjectDir + "results_rt/so.csv");
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
