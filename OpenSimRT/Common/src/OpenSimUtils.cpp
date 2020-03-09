#include "OpenSimUtils.h"

#include <Common/TimeSeriesTable.h>

using OpenSim::Model, OpenSim::Actuator, OpenSim::Storage,
        OpenSim::TimeSeriesTable;
using std::vector, std::string;
using namespace OpenSimRT;

int OpenSimUtils::generateUID() {
    static int id = 0;
    return id++;
}

vector<string>
OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(const Model& modelOther) {
    // get a working copy
    Model model(modelOther);
    model.initSystem();
    // get coordinates (model must be realized)
    vector<string> coordinateNames;
    const auto& coordinates = model.getCoordinatesInMultibodyTreeOrder();
    for (const auto& coordinate : coordinates) {
        coordinateNames.push_back(coordinate->getName());
    }
    return coordinateNames;
}

std::vector<std::string> OpenSimUtils::getCoordinateNames(const Model& model) {
    vector<string> coordinateNames;
    const auto& coordinateSet = model.getCoordinateSet();
    for (int i = 0; i < coordinateSet.getSize(); i++) {
        coordinateNames.push_back(coordinateSet[i].getName());
    }
    return coordinateNames;
}

vector<string> OpenSimUtils::getMuscleNames(const Model& model) {
    vector<string> muscleNames;
    for (int i = 0; i < model.getMuscles().getSize(); ++i) {
        muscleNames.push_back(model.getMuscles()[i].getName());
    }
    return muscleNames;
}

vector<string> OpenSimUtils::getActuatorNames(const Model& model) {
    vector<string> actuatorNames;
    for (int i = 0; i < model.getActuators().getSize(); ++i) {
        actuatorNames.push_back(model.getActuators()[i].getName());
    }
    return actuatorNames;
}

void OpenSimUtils::disableActuators(OpenSim::Model& model) {
    for (int i = 0; i < model.updActuators().getSize(); i++) {
        model.updActuators()[i].set_appliesForce(false);
    }
}

void OpenSimUtils::removeActuators(OpenSim::Model& model) {
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

TimeSeriesTable OpenSimUtils::getMultibodyTreeOrderedCoordinatesFromStorage(
        const Model& model, const std::string stoFilePath,
        double samplingInterval) {
    // load storage, convert to radians and re-sample
    Storage q(stoFilePath);
    if (q.isInDegrees()) {
        std::cout << "convert storage to radians" << std::endl;
        model.getSimbodyEngine().convertDegreesToRadians(q);
    }
    // re-sample after conversation, because Storage metadata (isInDegrees) are
    // lost afterwards (OpenSim bug)
    q.resampleLinear(samplingInterval);

    // set time column
    double* timeData = new double[q.getSize()];
    q.getTimeColumn(timeData);
    TimeSeriesTable table(
            std::vector<double>(timeData, timeData + q.getSize()));
    delete[] timeData;

    // build table columns
    const auto& coordinateNames =
            OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);
    const auto& columnLabels = q.getColumnLabels();
    for (const auto& coordinate : coordinateNames) {
        for (int i = 0; i < columnLabels.size(); ++i) {
            if (columnLabels[i].find(coordinate) != std::string::npos) {
                double* columnData = new double[q.getSize()];
                q.getDataColumn(columnLabels[i], columnData);
                table.appendColumn(columnLabels[i],
                                   SimTK::Vector(q.getSize(), columnData));
                delete[] columnData;
            }
        }
    }

    return table;
}
