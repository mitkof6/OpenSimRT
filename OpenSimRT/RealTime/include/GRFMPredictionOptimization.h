#ifndef GRFM_PREDICTION_OPTIMIZATION_H
#define GRFM_PREDICTION_OPTIMIZATION_H

#include "internal/RealTimeExports.h"

#include <OpenSim/Actuators/osimActuators.h>
#include <OpenSim/Common/osimCommon.h>
#include <OpenSim/OpenSim.h>
#include <OpenSim/Simulation/Model/Analysis.h>
#include <OpenSim/Simulation/Model/Force.h>
#include <OpenSim/Simulation/osimSimulation.h>
#include <Simulation/SimbodyEngine/Body.h>
#include <vector>

namespace OpenSimRT {

// custom contact force element
//==============================================================================
class RealTime_API ContactForceActuator {
 public:
    struct ContactWrench {
        SimTK::Vec3 force;
        SimTK::Vec3 moment;
        SimTK::Vec3 point;
    };

    ContactForceActuator();
    ContactForceActuator(const OpenSim::Model&, const std::string&,
                         const SimTK::Vec3&, const SimTK::Vec3&,
                         const std::string&);

    SimTK::Vector getMaxMultipliers() const;
    SimTK::Vector getMinMultipliers() const;
    void setOptimalForce(double force);
    double getOptimalForce() const;
    static int getNumMultipliers();
    static int getNumMultiplierConstraints();
    void setName(const std::string&);
    std::string getName() const;

    void computeForceVectors(const SimTK::State& s,
                             const SimTK::Vector& multipliers,
                             ContactWrench& planeWrench,
                             ContactWrench& bodyWrench) const;
    // void computeForceGradient(const SimTK::State& s,
    //                           const SimTK::Vector& multipliers) const;

 public:
    std::string name;
    double optimal_force;
    double static_friction;
    double kinetic_friction;
    double contact_tolerance;
    double contact_transition_zone;
    double v_trans;
    std::string plane_body;
    SimTK::Vec3 plane_origin;
    SimTK::Vec3 plane_normal;
    std::string point_body;
    SimTK::Vec3 point_location;

 protected:
    SimTK::Vector maxMultipliers;
    SimTK::Vector minMultipliers;
    // SimTK::ReferencePtr<const OpenSim::Body> planeBody;
    SimTK::ReferencePtr<const OpenSim::Body> pointBody;
};

// target forward declaration
//==============================================================================
class ContactForceAnalysisTarget;

// analysis
//==============================================================================
class RealTime_API ContactForceAnalysis {
    friend class ContactForceAnalysisTarget;

 public:
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qd;
        SimTK::Vector qdd;
    };
    struct Output {
        double t;
        SimTK::Vec3 point;
        SimTK::Vec3 force;
        SimTK::Vec3 moment;
        SimTK::Vector asVector();
    };
    struct Parameters {
        double convergence_tolerance;
        double constraint_tolerance;
        SimTK::Vec3 platform_offset;
    };

    ContactForceAnalysis(const OpenSim::Model& otherModel,
                         const Parameters& optimizationParameters);
    std::vector<Output> solve(const Input& input);

 private:
    int numContactMultipliers;
    OpenSim::Model model;
    SimTK::Vector parameterSeeds;
    std::vector<ContactForceActuator*> contacts;
    SimTK::ReferencePtr<SimTK::Optimizer> optimizer;
    SimTK::ReferencePtr<ContactForceAnalysisTarget> problem;
};

// target
//==============================================================================
class RealTime_API ContactForceAnalysisTarget : public SimTK::OptimizerSystem {
 public:
    ContactForceAnalysisTarget(OpenSim::Model* otherModel,
                               ContactForceAnalysis* otherAnalysis);

    void initParameters(SimTK::Vector& parameterSeeds, double&&) const;
    void prepareForOptimization(const ContactForceAnalysis::Input& input) const;
    SimTK::Vector_<SimTK::SpatialVec>
    getContactForces(const SimTK::Vector& multipliers) const;

 protected:
    int constraintFunc(const SimTK::Vector& parameters, bool new_parameters,
                       SimTK::Vector& errors) const override;
    int constraintJacobian(const SimTK::Vector& parameters, bool new_parameters,
                           SimTK::Matrix& jacobian) const override;
    int objectiveFunc(const SimTK::Vector& parameters, bool new_parameters,
                      SimTK::Real& objective) const override;
    int gradientFunc(const SimTK::Vector& parameters, bool new_parameters,
                     SimTK::Vector& gradient) const override;

 private:
    mutable SimTK::Vector tau;
    mutable SimTK::State state;

    SimTK::ReferencePtr<OpenSim::Model> model;
    SimTK::ReferencePtr<ContactForceAnalysis> analysis;
    std::multimap<std::string, ContactForceActuator*> contactsPerBodyMap;
};

} // namespace OpenSimRT

#endif