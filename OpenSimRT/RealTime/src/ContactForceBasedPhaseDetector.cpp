#include "ContactForceBasedPhaseDetector.h"

#include "Exception.h"
#include "GRFMPrediction.h"
#include "OpenSimUtils.h"

#include <OpenSim/Simulation/Model/ContactHalfSpace.h>
#include <OpenSim/Simulation/Model/ContactSphere.h>
#include <OpenSim/Simulation/SimbodyEngine/WeldJoint.h>
#include <SimTKcommon/internal/Stage.h>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

ContactForceBasedPhaseDetector::ContactForceBasedPhaseDetector(
        const Model& otherModel, const Parameters& otherParameters)
        : GaitPhaseDetector(otherParameters.windowSize),
          model(*otherModel.clone()), parameters(otherParameters) {
    // add platform
    auto platform = new OpenSim::Body("Platform", 1.0, Vec3(0), Inertia(0));
    model.addBody(platform);

    // add weld joint
    auto platformToGround = new WeldJoint("PlatformToGround", model.getGround(),
                                          Vec3(0), Vec3(0), *platform,
                                          -parameters.plane_origin, Vec3(0));
    model.addJoint(platformToGround);

    // add contact half-space
    auto platformContact = new ContactHalfSpace();
    platformContact->setName("PlatformContact");
    platformContact->set_location(Vec3(0));
    platformContact->set_orientation(Vec3(0.0, 0.0, -Pi / 2.0));
    platformContact->setFrame(*platform);
    model.addContactGeometry(platformContact);

    // add contact spheres
    auto rightHeelContact = new ContactSphere();
    auto leftHeelContact = new ContactSphere();
    auto rightToeContact = new ContactSphere();
    auto leftToeContact = new ContactSphere();
    rightHeelContact->set_location(parameters.rHeelSphereLocation);
    leftHeelContact->set_location(parameters.lHeelSphereLocation);
    rightToeContact->set_location(parameters.rToeSphereLocation);
    leftToeContact->set_location(parameters.lToeSphereLocation);
    rightHeelContact->setFrame(
            model.getBodySet().get(parameters.rFootBodyName));
    leftHeelContact->setFrame(model.getBodySet().get(parameters.lFootBodyName));
    rightToeContact->setFrame(model.getBodySet().get(parameters.rFootBodyName));
    leftToeContact->setFrame(model.getBodySet().get(parameters.lFootBodyName));
    rightHeelContact->setRadius(parameters.sphereRadius);
    leftHeelContact->setRadius(parameters.sphereRadius);
    rightToeContact->setRadius(parameters.sphereRadius);
    leftToeContact->setRadius(parameters.sphereRadius);
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
    model.addForce(rightContactForce.get());
    model.addForce(leftContactForce.get());

    // initialize system
    state = model.initSystem();
}

void ContactForceBasedPhaseDetector::updDetector(
        const GRFMPrediction::Input& input) {
    OpenSimUtils::updateState(model, state, input.q, input.qDot);
    model.realizeDynamics(state);

    // compute contact forces
    auto rightContactWrench = rightContactForce.get()->getRecordValues(state);
    Vec3 rightContactForce(-rightContactWrench.get(0),
                           -rightContactWrench.get(1),
                           -rightContactWrench.get(2));

    auto leftContactWrench = leftContactForce.get()->getRecordValues(state);
    Vec3 leftContactForce(-leftContactWrench.get(0), -leftContactWrench.get(1),
                          -leftContactWrench.get(2));

    // update detector internal state
    updDetectorState(input.t, rightContactForce.norm() - parameters.threshold,
                     leftContactForce.norm() - parameters.threshold);
}
