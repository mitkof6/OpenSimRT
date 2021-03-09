#include "ViconDataStream.h"

#include <chrono>
#include <functional>
#include <map>
#include <thread>

using namespace std;
using namespace SimTK;
using namespace OpenSimRT;

using namespace ViconDataStreamSDK::CPP;

/*******************************************************************************/

ViconDataStream::ViconDataStream(vector<Vec3> labForcePlatePositions)
        : labForcePlatePositions(labForcePlatePositions) {
    previousMarkerDataTime = -1.0;
    previousForceDataTime = -1.0;
    shouldTerminate = false;
}

void ViconDataStream::connect(string hostName) {
    while (!client.IsConnected().Connected) {
        if (client.Connect(hostName).Result != Result::Success) {
            cout << "warning - connect failed..." << endl;
            this_thread::sleep_for(chrono::milliseconds(100));
        }
    }
    cout << "connected to Vicon server at: " << hostName << endl;
}

void ViconDataStream::initialize(Direction::Enum xAxis, Direction::Enum yAxis,
                                 Direction::Enum zAxis) {
    // setup data
    client.EnableMarkerData();
    client.EnableDeviceData(); // grf
    client.DisableSegmentData();
    client.DisableUnlabeledMarkerData();

    // setup stream model
    while (client.SetStreamMode(StreamMode::ClientPull).Result !=
           Result::Success) {
        cout << ".";
    }
    cout << endl;

    // get force plates
    forcePlates = 0;
    while (forcePlates < 1) {
        cout << "waiting for devices..." << endl;
        client.GetFrame();
        auto forcePlatesInfo = client.GetForcePlateCount();
        if (forcePlatesInfo.Result == Result::Success) {
            forcePlates = forcePlatesInfo.ForcePlateCount;
        }
    }
    cout << "found " << forcePlates << " force plates" << endl;
    for (int i = 0; i < client.GetDeviceCount().DeviceCount; ++i) {
        if (client.GetDeviceName(i).DeviceType == DeviceType::ForcePlate) {
            string name = client.GetDeviceName(i).DeviceName;
            forcePlateNames.push_back(name);
            cout << name << endl;
        }
    }

    // get marker names
    bool frameComplete = false;
    do {
        if (client.GetFrame().Result == Result::Success) {
            int subjectCount = client.GetSubjectCount().SubjectCount;
            if (subjectCount != 1) {
                cout << "warning: " << subjectCount
                     << " subject(s) in the capture volume" << endl;
            } else {
                int subjectIndex = subjectCount - 1;
                string subjectName =
                        client.GetSubjectName(subjectIndex).SubjectName;
                int markerCount =
                        client.GetMarkerCount(subjectName).MarkerCount;
                for (int i = 0; i < markerCount; ++i) {
                    markerNames.push_back(
                            client.GetMarkerName(subjectName, i).MarkerName);
                }
                frameComplete = true;
            }
        }
        cout << ".";
    } while (!frameComplete);
    cout << "\n found " << markerNames.size() << " markers" << endl;

    client.SetAxisMapping(xAxis, yAxis, zAxis);
}

void ViconDataStream::getFrame() {
    // wait for frame
    while (client.GetFrame().Result != Result::Success) { cout << "."; }

    // frame number and frame rate
    auto frameNumber = client.GetFrameNumber().FrameNumber;
    static auto firstFrameNumber = frameNumber;
    auto frameRate = client.GetFrameRate();

    // get marker data
    double currentMarkerDataTime =
            1.0 / frameRate.FrameRateHz * (frameNumber - firstFrameNumber);
    if (currentMarkerDataTime > previousMarkerDataTime) {
        MarkerData markerData;
        markerData.time = currentMarkerDataTime;
        int subjectCount = client.GetSubjectCount().SubjectCount;
        if (subjectCount != 1) {
            cout << "warning: " << subjectCount
                 << " subject(s) in the capture volume" << endl;
        } else {
            int subjectIndex = subjectCount - 1;
            string subjectName =
                    client.GetSubjectName(subjectIndex).SubjectName;
            int markerCount = client.GetMarkerCount(subjectName).MarkerCount;
            for (int i = 0; i < markerCount; ++i) {
                string markerName =
                        client.GetMarkerName(subjectName, i).MarkerName;
                Output_GetMarkerGlobalTranslation markerGlobalTranslation =
                        client.GetMarkerGlobalTranslation(subjectName,
                                                          markerName);
                if (!markerGlobalTranslation.Occluded) {
                    // convert to meters
                    markerData.markers[markerName] = Vec3(
                            0.001 * markerGlobalTranslation.Translation[0],
                            0.001 * markerGlobalTranslation.Translation[1],
                            0.001 * markerGlobalTranslation.Translation[2]);
                } else {
                    markerData.markers[markerName] = Vec3(NaN);
                }
            }
        }
        markerBuffer.add(markerData);
        previousMarkerDataTime = currentMarkerDataTime;
    }

    // get force data
    auto forcePlateSubsamples =
            client.GetForcePlateSubsamples(0).ForcePlateSubsamples;
    for (int sample = 0; sample < forcePlateSubsamples; ++sample) {
        double currentForceDataTime = 1.0 / frameRate.FrameRateHz *
                                      (frameNumber - firstFrameNumber +
                                       1.0 / forcePlateSubsamples * sample);
        if (currentForceDataTime > previousForceDataTime) {
            ForceData forceData;
            forceData.time = currentForceDataTime;
            for (int i = 0; i < forcePlates; ++i) {
                Vec3 currentFpPos = labForcePlatePositions[i];
                Output_GetGlobalForceVector forceVector =
                        client.GetGlobalForceVector(i);
                Vec3 grfVec;
                grfVec[0] = forceVector.ForceVector[0];
                grfVec[1] = forceVector.ForceVector[1];
                grfVec[2] = forceVector.ForceVector[2];

                Output_GetGlobalCentreOfPressure centreOfPressure =
                        client.GetGlobalCentreOfPressure(i);
                Vec3 grfPoint;
                grfPoint[0] = centreOfPressure.CentreOfPressure[0];
                grfPoint[1] = centreOfPressure.CentreOfPressure[1];
                grfPoint[2] = centreOfPressure.CentreOfPressure[2];

                // calculate the values of the moment of the 'position'
                // reference system of the force plate in the global coordinate
                // system
                Vec3 momentOnPosition(0.);
                momentOnPosition[0] = currentFpPos[1] * grfVec[2] -
                                      currentFpPos[2] * grfVec[1];
                momentOnPosition[1] = currentFpPos[2] * grfVec[0] -
                                      currentFpPos[0] * grfVec[2];
                momentOnPosition[2] = currentFpPos[0] * grfVec[1] -
                                      currentFpPos[1] * grfVec[0];

                Output_GetGlobalMomentVector momentVector =
                        client.GetGlobalMomentVector(i);

                // calculate the correct values of the moments relatively the
                // global coordinate system by adding the missing position
                // moment
                Vec3 moments(0.0);
                moments[0] = momentVector.MomentVector[0] + momentOnPosition[0];
                moments[1] = momentVector.MomentVector[1] + momentOnPosition[1];
                moments[2] = momentVector.MomentVector[2] + momentOnPosition[2];

                Vec3 grfTorque;
                grfTorque[0] = 0;
                grfTorque[1] = (moments[1] - grfVec[0] * grfPoint[2] +
                                grfVec[2] * grfPoint[0]);
                grfTorque[2] = 0;

                if (abs(grfVec[1]) < 10) {
                    grfPoint[0] = 0;
                    grfPoint[2] = 0;
                }

                ExternalWrench forcePlateData;
                forcePlateData.force = -grfVec;
                forcePlateData.point = grfPoint;
                forcePlateData.torque = -grfTorque;
                forceData.externalWrenches[forcePlateNames[i]] = forcePlateData;
            }
            forceBuffer.add(forceData);
            previousForceDataTime = currentForceDataTime;
        }
    }
}

void ViconDataStream::startAcquisition() {
    function<void()> acquisitionFunction = [&]() -> void {
        while (!shouldTerminate) { getFrame(); }
    };
    thread acquisitionThread(acquisitionFunction);
    acquisitionThread.detach();
}

/*******************************************************************************/

Direction::Enum OpenSimRT::stringToDirection(std::string direction) {
    if (direction == "Up") {
        return Direction::Enum::Up;
    } else if (direction == "Down") {
        return Direction::Enum::Down;
    } else if (direction == "Left") {
        return Direction::Enum::Left;
    } else if (direction == "Right") {
        return Direction::Enum::Right;
    } else if (direction == "Forward") {
        return Direction::Enum::Forward;
    } else if (direction == "Backward") {
        return Direction::Enum::Backward;
    } else {
        THROW_EXCEPTION("unsupported direction: " + direction);
    }
}

/*******************************************************************************/
