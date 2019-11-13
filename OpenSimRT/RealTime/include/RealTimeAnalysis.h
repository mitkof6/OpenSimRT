/**
 * @file OActivePipeline.cpp
 *
 * \brief Real-time analysis for the estimation of kinematics, dynamics, muscle
 * forces and reaction loads.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef REAL_TIME_ANALYSIS_H
#define REAL_TIME_ANALYSIS_H

#include <iostream>
#include "InverseDynamics.h"
#include "InverseKinematics.h"
#include "JointReaction.h"
#include "MuscleOptimization.h"
#include "SimulationUtils.h"
#include "internal/RealTimeExports.h"

/**
 * \brief TODO
 */
struct RealTime_API MotionCaptureInput {
    InverseKinematics::Input ikFrame;
    std::vector<ExternalWrench::Input> externalWrenches;
};

/**
 * \brief A higher order function that is an interface between the motion
 * capture system and the InverseKinematics input. Termination of the
 * acquisition should be handled by throwing an exception.
 */
typedef std::function<MotionCaptureInput()> DataAcquisitionFunction;

/**
 * \brief TODO
 */
class RealTime_API RealTimeAnalysis {
 public:
 struct UnfilteredData {
     double t;
     SimTK::Vector q;
     std::vector<ExternalWrench::Input> externalWrenches;
     SimTK::Vector toVector();
     int size();
 };
 struct FilteredData {
     double t;
     SimTK::Vector q;
     SimTK::Vector qd;
     SimTK::Vector qdd;
     std::vector<ExternalWrench::Input> externalWrenches;
 };
 struct Parameters {
     bool useVisualizer;
     bool solveMuscleOptimization;
     double fc;          // cut of frequency
     double filterOrder; // order of the IIR filter
     int samplesDelay;
     std::string modelFile;
     std::vector<InverseKinematics::MarkerTask> ikMarkerTasks;
     std::vector<InverseKinematics::IMUTask> ikIMUTasks;
     double ikConstraintsWeight;
     std::vector<ExternalWrench::Parameters> wrenchParameters;
     MuscleOptimization::OptimizationParameters muscleOptimizationParameters;
     MomentArmFunction momentArmFunction;
     DataAcquisitionFunction dataAcquisitionFunction;
     std::vector<std::string> reactionForceOnBodies;
 } parameters;
 SimTK::ReferencePtr<InverseKinematics> inverseKinematics;
 SimTK::ReferencePtr<InverseDynamics> inverseDynamics;
 SimTK::ReferencePtr<MuscleOptimization> muscleOptimization;
 SimTK::ReferencePtr<JointReaction> jointReaction;
 SimTK::ReferencePtr<BasicModelVisualizer> visualizer;
 std::vector<ForceDecorator*> GRFDecorators;
 std::vector<ForceDecorator*> reactionForceDecorators;
 CircularBuffer<2000, UnfilteredData> buffer;
 SimTK::ReferencePtr<CSVLogger> qFiltered, qDotFiltered, qDDotFiltered;
 double previousAcquisitionTime;

 public:
 RealTimeAnalysis(const Parameters& parameters);
 void run();
 void exportResults(std::string dir);

 private:
 void acquisition();
 void processing();
 FilteredData filterData();
};

#endif
