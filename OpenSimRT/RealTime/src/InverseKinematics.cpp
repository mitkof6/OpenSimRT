/**
 * -----------------------------------------------------------------------------
 * Copyright 2019-2021 OpenSimRT developers.
 *
 * This file is part of OpenSimRT.
 *
 * OpenSimRT is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * OpenSimRT is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * OpenSimRT. If not, see <https://www.gnu.org/licenses/>.
 * -----------------------------------------------------------------------------
 */
#include "InverseKinematics.h"

#include "Exception.h"
#include "OpenSimUtils.h"

#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Tools/IKCoordinateTask.h>

using OpenSim::IKCoordinateTask;
using OpenSim::IKTaskSet;
using OpenSim::MarkerData;
using OpenSim::Model;
using OpenSim::TimeSeriesTable;
using OpenSim::Units;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using namespace SimTK;
using namespace OpenSimRT;

/******************************************************************************/

InverseKinematics::InverseKinematics(const OpenSim::Model& otherModel,
                                     const vector<MarkerTask>& markerTasks,
                                     const vector<IMUTask>& imuTasks,
                                     double constraintsWeight, double accuracy)
        : model(*otherModel.clone()), assembled(false) {
    // initialize model and assembler
    state = model.initSystem();
    assembler = new Assembler(model.getMultibodySystem());
    assembler->setAccuracy(accuracy);
    // assembler->setErrorTolerance(1e-3);
    assembler->setSystemConstraintsWeight(constraintsWeight);

    // populate assembly conditions for markers
    markerAssemblyConditions = new Markers();
    Array_<string> markerObservationOrder;
    for (const auto& task : markerTasks) {
        int markerIndex = model.getMarkerSet().getIndex(task.marker);
        if (markerIndex < 0) {
            THROW_EXCEPTION("marker: " + task.marker +
                            " does not exist in the "
                            "model");
        }
        const auto& marker = model.getMarkerSet()[markerIndex];
        const auto& mobod = marker.getParentFrame().getMobilizedBody();
        markerAssemblyConditions->addMarker(task.name, mobod,
                                            marker.get_location(), task.weight);
        markerObservationOrder.push_back(task.name);
    }
    markerAssemblyConditions->defineObservationOrder(markerObservationOrder);
    if (markerObservationOrder.size() != 0) {
        assembler->adoptAssemblyGoal(markerAssemblyConditions.get());
    }

    // populate assembly conditions for markers
    imuAssemblyConditions = new OrientationSensors();
    Array_<string> imuObservationOrder;
    for (const auto& task : imuTasks) {
        int bodyIndex = model.getBodySet().getIndex(task.body);
        if (bodyIndex < 0) {
            THROW_EXCEPTION("body: " + task.body + " does not exist in model");
        }
        const auto& body = model.getBodySet()[bodyIndex];
        const auto& mobod = body.getMobilizedBody();
        imuAssemblyConditions->addOSensor(
                task.name, mobod, task.orientation, // orientationInB = R_BS
                task.weight);
        imuObservationOrder.push_back(task.name);
    }
    imuAssemblyConditions->defineObservationOrder(imuObservationOrder);
    if (imuObservationOrder.size() != 0) {
        assembler->adoptAssemblyGoal(imuAssemblyConditions.get());
    }

    assembler->initialize(state);
}

InverseKinematics::Output InverseKinematics::solve(const Input& input) {
    state.updTime() = input.t;
    markerAssemblyConditions->moveAllObservations(input.markerObservations);
    imuAssemblyConditions->moveAllObservations(input.imuObservations);
    double rms;
    if (assembled) {
        rms = assembler->track();
    } else {
        rms = assembler->assemble();
        assembled = true;
    }
    assembler->updateFromInternalState(state);
    return InverseKinematics::Output{rms, input.t, state.getQ()};
}

TimeSeriesTable InverseKinematics::initializeLogger() {
    auto columnNames =
            OpenSimUtils::getCoordinateNamesInMultibodyTreeOrder(model);

    TimeSeriesTable q;
    q.setColumnLabels(columnNames);
    return q;
}

/******************************************************************************/

void InverseKinematics::createMarkerTasksFromIKTaskSet(
        const Model& model, const IKTaskSet& ikTaskSet,
        vector<MarkerTask>& markerTasks, vector<string>& observationOrder) {
    if (ikTaskSet.getSize() == 0) { THROW_EXCEPTION("IKTaskSet is empty"); }

    for (int i = 0; i < ikTaskSet.getSize(); ++i) {
        if (dynamic_cast<const IKCoordinateTask*>(&ikTaskSet[i])) {
            cout << "TODO - IKCoordinateTask:" << ikTaskSet[i].getName()
                 << " ignored" << endl;
            continue;
        }
        if (ikTaskSet[i].getApply() &&
            model.getMarkerSet().getIndex(ikTaskSet[i].getName()) >= 0) {
            markerTasks.push_back({ikTaskSet[i].getName(),
                                   ikTaskSet[i].getName(),
                                   ikTaskSet[i].getWeight()});
            observationOrder.push_back(ikTaskSet[i].getName());
        } else {
            cout << "marker: " << ikTaskSet[i].getName()
                 << " not in model, thus skipped from tracking" << endl;
        }
    }
}

void InverseKinematics::createMarkerTasksFromMarkerData(
        const Model& model, const MarkerData& markerData,
        vector<MarkerTask>& markerTasks, vector<string>& observationOrder) {
    for (int i = 0; i < markerData.getNumMarkers(); ++i) {
        auto markerName = markerData.getMarkerNames()[i];
        if (model.getMarkerSet().getIndex(markerName) >= 0) {
            markerTasks.push_back({markerName, markerName, 1.0});
            observationOrder.push_back(markerName);
        } else {
            cout << "marker: " + markerName
                 << " does not exist in model, thus skipped from tracking"
                 << endl;
        }
    }
}

void InverseKinematics::createMarkerTasksFromMarkerNames(
        const Model& model, const vector<string>& markerNames,
        vector<MarkerTask>& markerTasks, vector<string>& observationOrder) {
    for (int i = 0; i < markerNames.size(); ++i) {
        auto markerName = markerNames[i];
        if (model.getMarkerSet().getIndex(markerName) >= 0) {
            markerTasks.push_back({markerName, markerName, 1.0});
            observationOrder.push_back(markerName);
        } else {
            cout << "marker: " + markerName
                 << " does not exist in model, thus skipped from tracking"
                 << endl;
        }
    }
}

void InverseKinematics::createIMUTasksFromMarkerData(
        const Model& model, const MarkerData& markerData,
        vector<IMUTask>& imuTasks, vector<string>& observationOrder) {
    for (int i = 0; i < markerData.getNumMarkers(); ++i) {
        auto body = markerData.getMarkerNames()[i];
        if (model.getBodySet().getIndex(body) >= 0) {
            imuTasks.push_back({body, body, Rotation(), 1.0});
            observationOrder.push_back(body);
        } else {
            cout << "imu: " + body
                 << " does not exist in model, thus skipped from tracking"
                 << endl;
        }
    }
}

void InverseKinematics::createIMUTasksFromObservationOrder(
        const Model& model, const vector<string>& observationOrder,
        vector<IMUTask>& imuTasks) {
    for (const auto& frameName : observationOrder) {
        // get the frame with the attached IMU (includes PhysicalFrameOffsets)
        const auto& frame =
                model.findComponent<OpenSim::PhysicalFrame>(frameName);

        if (frame) { // if the frame exists..
            // get the base frame name
            const auto& bodyName = frame->findBaseFrame().getName();

            // create and push to imuTasks
            imuTasks.push_back({bodyName, bodyName,
                                frame->findTransformInBaseFrame().R(), 1.0});
        } else {
            cout << "imu: " + frameName
                 << " does not exist in model, thus skipped from tracking"
                 << endl;
        }
    }
}

InverseKinematics::Input InverseKinematics::getFrameFromMarkerData(
        int i, MarkerData& markerData, const vector<string>& observationOrder,
        bool isIMU) {
    // ensure results are in meters
    if (!isIMU && markerData.getUnits().getType() != Units::Meters) {
        markerData.convertToUnits(Units::Meters);
    }
    // if isIMU ensure that results are in radians
    if (isIMU && markerData.getUnits().getType() != Units::Radians) {
        markerData.convertToUnits(Units::Radians);
    }
    // extract based on observation order
    auto frame = markerData.getFrame(i);
    InverseKinematics::Input input;
    input.t = markerData.getFrame(i).getFrameTime();
    for (auto name : observationOrder) {
        int index = markerData.getMarkerNames().findIndex(name);
        auto vec = frame.getMarkers()[index];
        if (!isIMU) {
            input.markerObservations.push_back(vec);
        } else {
            input.imuObservations.push_back(Rotation(
                    BodyOrSpaceType::SpaceRotationSequence, vec[0],
                    SimTK::XAxis, vec[1], SimTK::YAxis, vec[2], SimTK::ZAxis));
        }
    }
    return input;
}

/******************************************************************************/
