#include "PhaseDetector.h"

#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

ContactForceBasedPhaseDetector::ContactForceBasedPhaseDetector(
        const Model& otherModel,
        const GRFMPrediction::Parameters& otherParameters)
        : model(*otherModel.clone()), Ths(-1), parameters(otherParameters),
          Tto(-1), Tds(-1), Tss(-1) {
    // initialize and set size of legphase sliding window
    phaseWindowR.init({GaitPhaseState::LegPhase::INVALID,
                       GaitPhaseState::LegPhase::INVALID});
    phaseWindowL.init({GaitPhaseState::LegPhase::INVALID,
                       GaitPhaseState::LegPhase::INVALID});

    // init leading leg state
    leadingLeg = GaitPhaseState::LeadingLeg::INVALID;

    //
    // add elements to a copy of the original model
    // ...

    // platform
    auto platform = new OpenSim::Body("Platform", 1.0, Vec3(0), Inertia(0));
    model.addBody(platform);

    // weld joint
    auto platformToGround = new WeldJoint(
            "PlatformToGround", model.getGround(), Vec3(0), Vec3(0), *platform,
            -parameters.contact_plane_origin, Vec3(0));
    model.addJoint(platformToGround);

    // contact half-space
    auto platformContact = new ContactHalfSpace();
    platformContact->setName("PlatformContact");
    platformContact->setLocation(Vec3(0));
    platformContact->setOrientation(Vec3(0.0, 0.0, -Pi / 2.0));
    platformContact->setBody(*platform);
    model.addContactGeometry(platformContact);

    // contact spheres
    auto rightHeelContact = new ContactSphere();
    auto leftHeelContact = new ContactSphere();
    auto rightToeContact = new ContactSphere();
    auto leftToeContact = new ContactSphere();
    rightHeelContact->setLocation(Vec3(0.012, -0.0015, -0.005));
    leftHeelContact->setLocation(Vec3(0.012, -0.0015, 0.005));
    rightToeContact->setLocation(Vec3(0.055, 0.01, -0.01));
    leftToeContact->setLocation(Vec3(0.055, 0.01, 0.01));
    rightHeelContact->setBody(model.getBodySet().get("calcn_r"));
    leftHeelContact->setBody(model.getBodySet().get("calcn_l"));
    rightToeContact->setBody(model.getBodySet().get("toes_r"));
    leftToeContact->setBody(model.getBodySet().get("toes_l"));
    rightHeelContact->setRadius(0.01);
    leftHeelContact->setRadius(0.01);
    rightToeContact->setRadius(0.01);
    leftToeContact->setRadius(0.01);
    rightHeelContact->setName("RHeelContact");
    leftHeelContact->setName("LHeelContact");
    rightToeContact->setName("RToeContact");
    leftToeContact->setName("LToeContact");
    model.addContactGeometry(rightHeelContact);
    model.addContactGeometry(leftHeelContact);
    model.addContactGeometry(rightToeContact);
    model.addContactGeometry(leftToeContact);

    // contact parameters
    double stiffness = 2e6;
    double dissipation = 1.0;
    double staticFriction = 0.9;
    double dynamicFriction = 0.8;
    double viscousFriction = 0.6;

    auto rightContactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
            stiffness, dissipation, staticFriction, dynamicFriction,
            viscousFriction);
    auto leftContactParams = new OpenSim::HuntCrossleyForce::ContactParameters(
            stiffness, dissipation, staticFriction, dynamicFriction,
            viscousFriction);

    rightContactParams->addGeometry("PlatformContact");
    rightContactParams->addGeometry("RHeelContact");
    rightContactParams->addGeometry("RToeContact");
    leftContactParams->addGeometry("PlatformContact");
    leftContactParams->addGeometry("LHeelContact");
    leftContactParams->addGeometry("LToeContact");

    // contact forces
    rightContactForce = new OpenSim::HuntCrossleyForce(rightContactParams);
    leftContactForce = new OpenSim::HuntCrossleyForce(leftContactParams);
    rightContactForce->setName("RightContactForce");
    leftContactForce->setName("LeftContactForce");
    model.addForce(rightContactForce);
    model.addForce(leftContactForce);

    // initialize system
    state = model.initSystem();

    // define function for detecting HS - transition SWING -> STANCE
    detectHS = [](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.data[0] == GaitPhaseState::LegPhase::SWING &&
                w.data[1] == GaitPhaseState::LegPhase::STANCE)
                       ? true
                       : false;
    };

    // define function for detecting TO - transition STANCE -> SWING
    detectTO = [](const SlidingWindow<GaitPhaseState::LegPhase>& w) {
        return (w.data[0] == GaitPhaseState::LegPhase::STANCE &&
                w.data[1] == GaitPhaseState::LegPhase::SWING)
                       ? true
                       : false;
    };
}

void ContactForceBasedPhaseDetector::updDetector(
        const GRFMPrediction::Input& input) {
    // update model state
    updateState(input, model, state, Stage::Dynamics);

    // update leg phase
    auto phaseR = updLegPhase(rightContactForce);
    auto phaseL = updLegPhase(leftContactForce);

    // push to sliding window
    phaseWindowR.insert(phaseR);
    phaseWindowL.insert(phaseL);

    // udpate time constants
    Tto = (detectTO(phaseWindowR) || detectTO(phaseWindowL)) ? state.getTime()
                                                             : Tto;
    Ths = (detectHS(phaseWindowR) || detectHS(phaseWindowL)) ? state.getTime()
                                                             : Ths;
    Tds = (Tto > Ths && Ths > 0) ? Tto - Ths : Tds;
    Tss = (Ths > Tto && Tto > 0) ? Ths - Tto : Tss;

    // udpate leading leg
    leadingLeg = [&]() {
        if (detectHS(phaseWindowR))
            return GaitPhaseState::LeadingLeg::RIGHT;
        else if (detectHS(phaseWindowL))
            return GaitPhaseState::LeadingLeg::LEFT;
        else
            return leadingLeg; // yield previous value
    }();

    // update gait phase
    gaitPhase = updGaitPhase(phaseR, phaseL);
}

GaitPhaseState::GaitPhase ContactForceBasedPhaseDetector::updGaitPhase(
        const GaitPhaseState::LegPhase& phaseR,
        const GaitPhaseState::LegPhase& phaseL) {
    // determine current gait phase based on leg phase
    GaitPhaseState::GaitPhase phase;
    if (phaseR == GaitPhaseState::LegPhase::STANCE &&
        phaseL == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::DOUBLE_SUPPORT;

    } else if (phaseL == GaitPhaseState::LegPhase::SWING &&
               phaseR == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::LEFT_SWING;

    } else if (phaseR == GaitPhaseState::LegPhase::SWING &&
               phaseL == GaitPhaseState::LegPhase::STANCE) {
        phase = GaitPhaseState::GaitPhase::RIGHT_SWING;

    } else {
        phase = GaitPhaseState::GaitPhase::INVALID;
    }
    return phase;
}

GaitPhaseState::LegPhase ContactForceBasedPhaseDetector::updLegPhase(
        const OpenSim::HuntCrossleyForce* contact) {
    // compute contact force
    auto contactWrench = contact->getRecordValues(state);
    Vec3 contactForce(-contactWrench[0], -contactWrench[1], -contactWrench[2]);

    // f > threshold
    return [](const double& f) {
        if (f > 0)
            return GaitPhaseState::LegPhase::STANCE;
        else if (f <= 0)
            return GaitPhaseState::LegPhase::SWING;
        else
            return GaitPhaseState::LegPhase::INVALID;
    }(contactForce.norm() - parameters.stance_threshold);
}

bool ContactForceBasedPhaseDetector::isDetectorReady() {
    // detector is ready when time constants, gait phase and leading leg
    // member variables are valid
    return (Ths >= 0 && Tto >= 0 && Tds >= 0 && Tss >= 0 &&
            leadingLeg != GaitPhaseState::LeadingLeg::INVALID &&
            gaitPhase != GaitPhaseState::GaitPhase::INVALID)
                   ? true
                   : false;
}
