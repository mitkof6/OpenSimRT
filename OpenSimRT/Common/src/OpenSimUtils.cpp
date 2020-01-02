#include "OpenSimUtils.h"

using OpenSim::Model, OpenSim::Actuator;
using std::vector, std::string;
using namespace OpenSimRT;

int ModelUtils::generateUID() {
    static int id = 0;
    return id++;
}

vector<string> ModelUtils::getCoordinateNames(const Model& model) {
    vector<string> coordinateColumnNames;
    const auto& coordinates = model.getCoordinatesInMultibodyTreeOrder();
    for (const auto& coordinate : coordinates) {
        coordinateColumnNames.push_back(coordinate->getName());
    }
    return coordinateColumnNames;
}

vector<string> ModelUtils::getMuscleNames(const Model& model) {
    vector<string> muscleColumnNames;
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        muscleColumnNames.push_back(model.getMuscles()[i].getName());
    }
    return muscleColumnNames;
}

vector<string> ModelUtils::getActuatorNames(const Model& model) {
    vector<string> actuatorColumnNames;
    for (int i = 0; i < model.getActuators().getSize(); ++i) {
        actuatorColumnNames.push_back(model.getActuators()[i].getName());
    }
    return actuatorColumnNames;
}

void ModelUtils::disableActuators(OpenSim::Model& model) {
    for (int i = 0; i < model.updActuators().getSize(); i++) {
        model.updActuators()[i].set_appliesForce(false);
    }
}

void ModelUtils::removeActuators(OpenSim::Model& model) {
    // save a list of pointers of the actuators to delete
    std::vector<Actuator*> actuatorsToDelete;
    auto& actuatorSet = model.updActuators();
    for (int i = 0; i < actuatorSet.getSize(); ++i) {
        actuatorsToDelete.push_back(&actuatorSet.get(i));
    }
    // delete the actuators
    for (const auto* act : actuatorsToDelete) {
        int index = model.getForceSet().getIndex(act, 0);
        model.updForceSet().remove(index);
    }
}
