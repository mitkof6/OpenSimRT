#ifndef GROUND_REACTION_FORCE_PREDICTION
#define GROUND_REACTION_FORCE_PREDICTION

#include "internal/RealTimeExports.h"

#include <OpenSim/Simulation/Model/Model.h>

namespace OpenSimRT {

class RealTime_API ContactForceBasedPhaseDetector;

// helper function for updating opensim state
template <typename T>
void updateState(const T& input, const OpenSim::Model& model,
                 SimTK::State& state, const SimTK::Stage& stage) {
    const auto& coordinateSet = model.getCoordinatesInMultibodyTreeOrder();
    for (int i = 0; i < coordinateSet.size(); ++i) {
        coordinateSet[i]->setValue(state, input.q[i]);
        coordinateSet[i]->setSpeedValue(state, input.qDot[i]);
    }
    state.updTime() = input.t;
    model.getMultibodySystem().realize(state, stage);
}

// basic sliding window implementation
template <typename T> struct SlidingWindow {
    std::vector<T> data;  // sliding window data
    std::size_t capacity; // sliding window size

    // set initial values
    void init(std::vector<T>&& aData) {
        data = std::forward<std::vector<T>>(aData);
        capacity = data.size();
    }

    // insert element
    void insert(const T& x) {
        if (data.size() == capacity) data.erase(data.begin());
        data.push_back(x);
    }

    // reserve space in memory
    void setSize(const std::size_t& size) {
        capacity = size;
        data.reserve(size);
    }

    // compute mean value
    T mean() {
        return 1.0 * std::accumulate(data.begin(), data.end(), T()) /
               int(data.size());
    }
};
/*******************************************************************************/

// Possible gait related phases.
class RealTime_API GaitPhaseState {
 public:
    enum class LegPhase { INVALID, SWING, STANCE };

    enum class GaitPhase { INVALID, RIGHT_SWING, LEFT_SWING, DOUBLE_SUPPORT };

    enum class LeadingLeg { INVALID, RIGHT, LEFT };
};

// Ground Reaction Force & Moment Prediction Module
class RealTime_API GRFMPrediction {
    typedef std::function<double(const double&)> TransitionFuction;
    typedef std::function<SimTK::Vec3(const double&, const SimTK::Vec3&)>
            CoPTrajectory;

 public:
    struct Input {
        double t;
        SimTK::Vector q;
        SimTK::Vector qDot;
        SimTK::Vector qDDot;
    };

    struct Output {
        double t;
        SimTK::Vec3 point;
        SimTK::Vec3 force;
        SimTK::Vec3 moment;
        SimTK::Vector asVector();
    };

    struct Parameters {
        double stance_threshold; // contact-force threshold
        SimTK::Vec3 contact_plane_origin;
        SimTK::UnitVec3 contact_plane_normal;
    } parameters;

    // constructor
    GRFMPrediction(const OpenSim::Model&, const Parameters&);

    // compute the ground reaction forces, moments and cop
    std::vector<Output> solve(const Input& input);

 private:
    // transition functions based on the STA
    TransitionFuction anteriorForceTransition;
    TransitionFuction verticalForceTransition;
    TransitionFuction lateralForceTransition;
    TransitionFuction exponentialTransition;
    // TODO experiment with different functions
    // TODO implement for moments if required

    // function that describes the CoP trajectory during gait
    CoPTrajectory copPosition;

    double t, Tds, Tss; // current simulation time, double support and single
                        // support period

    // gait direction based on the average pelvis of the pelvis local frame
    SlidingWindow<SimTK::Vec3> gaitDirectionBuffer;

    OpenSim::Model model;
    SimTK::State state;

    // contact-force based phase detection
    SimTK::ReferencePtr<ContactForceBasedPhaseDetector> gaitPhaseDetector;
    GaitPhaseState::GaitPhase gaitphase; // gait phase during simulation

    // station points forming the cop trajectory
    SimTK::ReferencePtr<OpenSim::Station> heelStationR;
    SimTK::ReferencePtr<OpenSim::Station> heelStationL;
    SimTK::ReferencePtr<OpenSim::Station> toeStationR;
    SimTK::ReferencePtr<OpenSim::Station> toeStationL;

    // separate total reaction into R/L foot reaction components
    void seperateReactionComponents(
            const SimTK::Vec3& totalReactionComponent,
            const TransitionFuction& anteriorComponentFunction,
            const TransitionFuction& verticalComponentFunction,
            const TransitionFuction& lateralComponentFunction,
            SimTK::Vec3& rightReactionComponent,
            SimTK::Vec3& leftReactionComponent);

    // compute the CoP on each foot
    void computeReactionPoint(SimTK::Vec3& rightPoint, SimTK::Vec3& leftPoint);
};
} // namespace OpenSimRT
#endif // !GROUND_REACTION_FORCE_PREDICTION
