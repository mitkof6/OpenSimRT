#include "RealTimeAnalysis.h"
#include <thread>
#include <OpenSim/Common/Signal.h>
#include <OpenSim/Common/GCVSpline.h>
#include <OpenSim/Simulation/Model/BodySet.h>

using namespace std;
using namespace OpenSim;
using namespace SimTK;

void updateReactionForceDecorator(const Model& model, const State& state,
                                  const Vector_<SpatialVec>& reactionWrench,
                                  const string& reactionOnBody,
                                  ForceDecorator* reactionForceDecorator) {
    auto bodyIndex = model.getBodySet().getIndex(reactionOnBody, 0);
    const auto& body = model.getBodySet()[bodyIndex];
    auto force = -reactionWrench[bodyIndex](1); // mirror force (1)
    Vec3 joint(0);
    model.getSimbodyEngine().transformPosition(state,
                                               body, Vec3(0),
                                               model.getGround(),
                                               joint);
    reactionForceDecorator->update(joint, force);
}

Vector RealTimeAnalysis::UnfilteredData::toVector() {
    if (size() == 0) {
        THROW_EXCEPTION("cannot convert from empty");
    }

    Vector temp(size());
    int index = 0;
    temp[index++] = t;
    for (int i = 0; i < q.size(); ++i) {
        temp[index++] = q[i];
    }

    for (int i = 0; i < externalWrenches.size(); ++i) {
        auto tempVec = externalWrenches[i].toVector();
        for (int j = 0; j < tempVec.size(); ++j) {
            temp[index++] = tempVec[j];
        }
    }

    return temp;
}

int RealTimeAnalysis::UnfilteredData::size() {
    return 1 + q.size() + externalWrenches.size() * ExternalWrench::Input::size();
}

RealTimeAnalysis::RealTimeAnalysis(const RealTimeAnalysis::Parameters& parameters)
    : parameters(parameters), previousAcquisitionTime(-1.0) {

    inverseKinematics = new InverseKinematics(parameters.modelFile,
                                              parameters.ikConstraintsWeight,
                                              parameters.ikMarkerTasks,
                                              parameters.ikIMUTasks);

    inverseDynamics = new InverseDynamics(parameters.modelFile,
                                          parameters.wrenchParameters);

    muscleOptimization = new MuscleOptimization(parameters.modelFile,
                                                parameters.muscleOptimizationParameters,
                                                parameters.momentArmFunction,
                                                new TorqueBasedTargetNonLinearMuscle());

    jointReaction = new JointReaction(parameters.modelFile,
                                      parameters.wrenchParameters);

    // visualization
    if (parameters.useVisualizer) {
        visualizer = new BasicModelVisualizer(parameters.modelFile);
        visualizer->visualizer->setWindowTitle("Real-time Analysis");
        visualizer->visualizer->setBackgroundType(Visualizer::BackgroundType::GroundAndSky);
        for (int i = 0; i < parameters.wrenchParameters.size(); ++i) {
            GRFDecorators.push_back(new ForceDecorator(Green, 0.001, 3));
            visualizer->visualizer->addDecorationGenerator(GRFDecorators.back());
        }
        for (int i = 0; i < parameters.reactionForceOnBodies.size(); ++i) {
            reactionForceDecorators.push_back(new ForceDecorator(Red, 0.0005, 3));
            visualizer->visualizer->addDecorationGenerator(reactionForceDecorators.back());
        }
    }

    // loggers
    auto coordinateColumnNames = getCoordinateNames(inverseKinematics->model);
    coordinateColumnNames.insert(coordinateColumnNames.begin(), "time");
    qFiltered = new CSVLogger(coordinateColumnNames);
    qDotFiltered = new CSVLogger(coordinateColumnNames);
    qDDotFiltered = new CSVLogger(coordinateColumnNames);
}

void RealTimeAnalysis::run() {
    thread acquisitionThread(&RealTimeAnalysis::acquisition, this);
    thread processingThread(&RealTimeAnalysis::processing, this);
    acquisitionThread.join();
    processingThread.join();
}

void RealTimeAnalysis::acquisition() {
    while (!visualizer->shouldTerminate) {
        try {
            // get data from Vicon or other method
            auto acquisitionData = parameters.dataAcquisitionFunction();
            if (previousAcquisitionTime >= acquisitionData.ikFrame.t) {
                continue;
            }

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
        } catch (exception &e) {
            cout << e.what() << endl;
            cout << "acquisition terminated" << endl;
            visualizer->shouldTerminate = true;
        }
    }
}

void RealTimeAnalysis::processing() {
    while (!visualizer->shouldTerminate) {
        auto filteredData = filterData();
        Vector am;
	Vector_<SpatialVec> reactionWrenches;

	// solve id
        auto id = inverseDynamics->solve({filteredData.t,
                                          filteredData.q,
                                          filteredData.qd,
                                          filteredData.qdd,
                                          filteredData.externalWrenches});

        if (parameters.solveMuscleOptimization) {
            auto so = muscleOptimization->solve({filteredData.t,
                                                 filteredData.q,
                                                 id.tau});
            am = so.am;

            auto jr = jointReaction->solve({filteredData.t,
                                            filteredData.q,
                                            filteredData.qd,
                                            so.fm,
                                            filteredData.externalWrenches});
	    reactionWrenches = jr.reactionWrench;
        }

        // visualization
        if (parameters.useVisualizer) {
            visualizer->update(filteredData.q, am);
	    for (int i = 0; i < filteredData.externalWrenches.size(); ++i) {
		GRFDecorators[i]->update(filteredData.externalWrenches[i].point,
					 filteredData.externalWrenches[i].force);
	    }
	    for (int i = 0; i < parameters.reactionForceOnBodies.size(); ++i) {
		updateReactionForceDecorator(visualizer->model,
					     visualizer->state,
					     reactionWrenches,
					     parameters.reactionForceOnBodies[i],
					     reactionForceDecorators[i]);
	    }

        }
    }
}

RealTimeAnalysis::FilteredData RealTimeAnalysis::filterData() {
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
            filteredData.qd[j - 1] = spline.calcDerivative({0}, Vector(1, filteredData.t));
            filteredData.qdd[j - 1] = spline.calcDerivative({0, 0}, Vector(1, filteredData.t));
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
    qFiltered->addRow(filteredData.t, filteredData.q);
    qDotFiltered->addRow(filteredData.t, filteredData.qd);
    qDDotFiltered->addRow(filteredData.t, filteredData.qdd);

    return filteredData;
}

void RealTimeAnalysis::exportResults(string dir) {
    inverseKinematics->logger->exportToFile(dir + "q_unfiltered.csv");
    qFiltered->exportToFile(dir + "q_filtered.csv");
    qDotFiltered->exportToFile(dir + "qDot_filtered.csv");
    qDDotFiltered->exportToFile(dir + "qDDot_filtered.csv");
    inverseDynamics->logger->exportToFile(dir + "tau.csv");
    for (int i = 0; i < inverseDynamics->externalWrenches.size(); ++i) {
        inverseDynamics->externalWrenches[i]->logger->exportToFile(
            dir + "wrench_" + toString(i) + ".csv");
    }
    muscleOptimization->logger->exportToFile(dir + "so.csv");
    jointReaction->logger->exportToFile(dir + "jr.csv");
}
