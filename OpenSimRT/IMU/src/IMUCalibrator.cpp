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
#include "IMUCalibrator.h"
#include "Exception.h"
#include <SimTKcommon/internal/Quaternion.h>

using namespace OpenSimRT;
using namespace OpenSim;
using namespace SimTK;
using namespace std;

void IMUCalibrator::setup(const std::vector<std::string>& observationOrder) {
    R_heading = SimTK::Rotation();
    R_GoGi = SimTK::Rotation();

    // copy observation order list
    imuBodiesObservationOrder = std::vector<std::string>(
            observationOrder.begin(), observationOrder.end());

    // initialize system
    state = model.initSystem();
    model.realizePosition(state);

    // get default model pose body orientation in ground
    for (const auto& label : imuBodiesObservationOrder) {
        const OpenSim::PhysicalFrame* frame = nullptr;
        if ((frame = model.findComponent<OpenSim::PhysicalFrame>(label))) {
            imuBodiesInGround[label] =
                    frame->getTransformInGround(state).R(); // R_GB
        }
    }
}

SimTK::Rotation IMUCalibrator::setGroundOrientationSeq(const double& xDegrees,
                                                       const double& yDegrees,
                                                       const double& zDegrees) {
    auto xRad = SimTK::convertDegreesToRadians(xDegrees);
    auto yRad = SimTK::convertDegreesToRadians(yDegrees);
    auto zRad = SimTK::convertDegreesToRadians(zDegrees);

    R_GoGi = Rotation(SimTK::BodyOrSpaceType::SpaceRotationSequence, xRad,
                      SimTK::XAxis, yRad, SimTK::YAxis, zRad, SimTK::ZAxis);
    return R_GoGi;
}

SimTK::Rotation
IMUCalibrator::computeHeadingRotation(const std::string& baseImuName,
                                      const std::string& imuDirectionAxis) {
    if (!imuDirectionAxis.empty() && !baseImuName.empty()) {
        // set coordinate direction based on given imu direction axis given as
        // string
        std::string imuAxis = IO::Lowercase(imuDirectionAxis);
        SimTK::CoordinateDirection baseHeadingDirection(SimTK::ZAxis);
        int direction = 1;
        if (imuAxis.front() == '-') direction = -1;
        const char& back = imuAxis.back();
        if (back == 'x')
            baseHeadingDirection =
                    SimTK::CoordinateDirection(SimTK::XAxis, direction);
        else if (back == 'y')
            baseHeadingDirection =
                    SimTK::CoordinateDirection(SimTK::YAxis, direction);
        else if (back == 'z')
            baseHeadingDirection =
                    SimTK::CoordinateDirection(SimTK::ZAxis, direction);
        else { // Throw, invalid specification
            THROW_EXCEPTION("Invalid specification of heading axis '" +
                            imuAxis + "' found.");
        }

        // find base imu body index in observation order
        auto baseBodyIndex = std::distance(
                imuBodiesObservationOrder.begin(),
                std::find(imuBodiesObservationOrder.begin(),
                          imuBodiesObservationOrder.end(), baseImuName));

        // get initial measurement of base imu
        const auto q0 = staticPoseQuaternions[baseBodyIndex];
        const auto base_R = R_GoGi * ~Rotation(q0);

        // get initial direction from the imu measurement (the axis looking
        // front)
        UnitVec3 baseSegmentXheading = base_R(baseHeadingDirection.getAxis());
        if (baseHeadingDirection.getDirection() < 0)
            baseSegmentXheading = baseSegmentXheading.negate();

        // get frame of imu body
        const PhysicalFrame* baseFrame = nullptr;
        if (!(baseFrame = model.findComponent<PhysicalFrame>(baseImuName))) {
            THROW_EXCEPTION(
                    "Frame of given body name does not exist in the model.");
        }

        // express unit x axis of local body frame to ground frame
        Vec3 baseFrameX = UnitVec3(1, 0, 0);
        const SimTK::Transform& baseXForm =
                baseFrame->getTransformInGround(state);
        Vec3 baseFrameXInGround = baseXForm.xformFrameVecToBase(baseFrameX);

        // compute the angular difference between the model heading and imu
        // heading
        auto angularDifference =
                acos(~baseSegmentXheading * baseFrameXInGround);

        // compute sign
        auto xproduct = baseFrameXInGround % baseSegmentXheading;
        if (xproduct.get(1) > 0) { angularDifference *= -1; }

        // set heading rotation (rotation about Y axis)
        R_heading = Rotation(angularDifference, SimTK::YAxis);

    } else {
        cout << "No heading correction is applied. Heading rotation is set to "
                "default"
             << endl;
    }

    return R_heading;
}

void IMUCalibrator::calibrateIMUTasks(
        vector<InverseKinematics::IMUTask>& imuTasks) {
    for (int i = 0; i < staticPoseQuaternions.size(); ++i) {
        const auto& q0 = staticPoseQuaternions[i];
        const auto R0 = R_heading * R_GoGi * ~Rotation(q0);

        const auto& bodyName = imuTasks[i].body;
        const auto R_BS = ~imuBodiesInGround[bodyName] * R0; // ~R_GB * R_GO

        imuTasks[i].orientation = std::move(R_BS);
    }
}

void IMUCalibrator::recordNumOfSamples(const size_t& numSamples) {
    impl->recordNumOfSamples(numSamples);
    staticPoseQuaternions = impl->computeAvgStaticPose();
}

void IMUCalibrator::recordTime(const double& timeout) {
    impl->recordTime(timeout);
    staticPoseQuaternions = impl->computeAvgStaticPose();
}
