#include "OpenSimUtils.h"

using OpenSim::Model;
using std::vector, std::string;
using namespace OpenSimRT;


int ModelUtils::generateUID() {
    static int id = 0;
    return id++;
}

vector<string> ModelUtils::getCoordinateNames(const Model& model) {
    vector<string> coordinateColumnNames;
    for (int i = 0; i < model.getCoordinateSet().getSize(); ++i) {
        coordinateColumnNames.push_back(model.getCoordinateSet()[i].getName());
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

vector<string> getActuatorNames(const Model& model) {
    vector<string> actuatorColumnNames;
    for (int i = 0; i < model.getActuators().getSize(); ++i) {
       actuatorColumnNames.push_back(model.getActuators()[i].getName());
    }
    return actuatorColumnNames;
}
