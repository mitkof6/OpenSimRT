#include "NGIMUCalibrator.h"

#include "Exception.h"

using namespace OpenSimRT;
using namespace OpenSim;
using namespace SimTK;
using namespace std;

NGNGIMUCalibrator::NGNGIMUCalibrator(const Model& otherModel,
                                     InputDriver<NGIMUData>* const driver,
                                     const vector<string>& observationOrder)
        : m_driver(driver), model(*otherModel.clone()), R_heading(Rotation()),
          R_GoGi(Rotation()) {
    // copy observation order list
    imuBodiesObservationOrder = std::vector<std::string>(
            observationOrder.begin(), observationOrder.end());

    // initialize system
    state = model.initSystem();
    model.realizePosition(state);

    // get default model pose body orientation in ground
    for (const auto& label : imuBodiesObservationOrder) {
        const PhysicalFrame* frame = nullptr;
        if ((frame = model.findComponent<PhysicalFrame>(label))) {
            imuBodiesInGround[label] =
                    frame->getTransformInGround(state).R(); // R_GB
        }
    }
}

SimTK::Rotation
NGNGIMUCalibrator::setGroundOrientationSeq(const double& xDegrees,
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
NGNGIMUCalibrator::computeHeadingRotation(const std::string& baseImuName,
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
        const auto& q0 = initIMUData[baseBodyIndex].quaternion.q;
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

void NGNGIMUCalibrator::calibrateIMUTasks(
        vector<InverseKinematics::IMUTask>& imuTasks) {
    for (int i = 0; i < initIMUData.size(); ++i) {
        const auto& q0 = initIMUData[i].quaternion.q;
        const auto R0 = R_heading * R_GoGi * ~Rotation(q0);

        const auto& bodyName = imuTasks[i].body;
        const auto R_BS = ~imuBodiesInGround[bodyName] * R0; // ~R_GB * R_GO

        imuTasks[i].orientation = std::move(R_BS);
    }
}

InverseKinematics::Input NGNGIMUCalibrator::transform(
        const std::pair<double, std::vector<NGIMUData>>& imuData,
        const SimTK::Array_<Vec3>& markerData) {
    InverseKinematics::Input input;

    // time
    input.t = imuData.first;

    // imu data
    for (int i = 0; i < imuData.second.size(); ++i) {
        const auto& q = imuData.second[i].quaternion.q; // current input
        const auto R = R_heading * R_GoGi * ~Rotation(q);
        input.imuObservations.push_back(R);
    }

    // marker data
    for (int i = 0; i < markerData.size(); ++i) {
        input.markerObservations.push_back(markerData[i]);
    }
    return input;
}

void NGNGIMUCalibrator::recordNumOfSamples(const size_t& numSamples) {
    cout << "Recording Static Pose..." << endl;
    size_t i = 0;
    while (i < numSamples) {
        // get frame measurements
        quatTable.push_back(m_driver->getData());
        ++i;
    }
    computeAvgStaticPose();
}
void NGNGIMUCalibrator::recordTime(const double& timeout) {
    cout << "Recording Static Pose..." << endl;
    const auto start = chrono::steady_clock::now();
    while (std::chrono::duration_cast<chrono::seconds>(
                   chrono::steady_clock::now() - start)
                   .count() < timeout) {
        // get frame measurements
        quatTable.push_back(m_driver->getData());
    }
    computeAvgStaticPose();
}

void NGNGIMUCalibrator::computeAvgStaticPose() {
    initIMUData = *(quatTable.end() - 1); // TODO: compute actual average
}
