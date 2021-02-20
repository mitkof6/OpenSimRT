/**
 * @file TestMarkerReconstruction.cpp
 *
 * @brief Test the missing marker reconstruction module.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#include "Exception.h"
#include "INIReader.h"
#include "InverseKinematics.h"
#include "MarkerReconstruction.h"
#include "Settings.h"

#include <Actuators/Thelen2003Muscle.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <iostream>

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

/**
 * Util function to simulate missing markers during a recorded trial.
 *
 * @param [t] current time of the recorder trial
 * @param [markerObservations] recorded markder positions
 * @param [markerLabels] marker names
 * @param [missingMarkerLabels] missing marker names
 * @param [duration] duration the missing markers are occluded
 * @param [t0] starting time of the occlusion
 */
Array_<Vec3> simulateMissingMarkers(const double& t,
                                    const Array_<Vec3>& markerObservations,
                                    const vector<string>& markerLabels,
                                    const vector<string>& missingMarkerLabels,
                                    const double& duration, const double& t0) {
    if (t0 == 0.0)
        THROW_EXCEPTION("Invalid input. Select init sample time > 0");

    Array_<Vec3> output(markerObservations);
    for (int i = 0; i < missingMarkerLabels.size(); ++i) {
        auto itr = find(markerLabels.begin(), markerLabels.end(),
                        missingMarkerLabels[i]);
        if (itr == markerLabels.end()) {
            THROW_EXCEPTION("Invalid input. Marker name does not exist");
        }
        if (itr != markerLabels.end() && t >= t0 && t - t0 <= duration) {
            int idx = distance(markerLabels.begin(), itr);
            output[idx] = Vec3(NaN, NaN, NaN);
        }
    }
    return output;
}

/**************************************************************************/

void run() {
    // subject data
    INIReader ini(INI_FILE);
    auto section = "TEST_MISSING_MARKER_RECONSTRUCTION_FROM_FILE";
    auto subjectDir = DATA_DIR + ini.getString(section, "SUBJECT_DIR", "");
    auto modelFile = subjectDir + ini.getString(section, "MODEL_FILE", "");
    auto trcFile = subjectDir + ini.getString(section, "TRC_FILE", "");

    // simulate-missing-markers parameters
    auto missingMarkerLabels =
            ini.getVector(section, "MISSING_MARKERS", vector<string>{});
    auto duration = ini.getReal(section, "OCCLUSION_DURATION", 0.0);
    auto t0 = ini.getReal(section, "OCCLUSION_INIT_TIME", 0.0);

    // setup model
    Object::RegisterType(Thelen2003Muscle());
    Model model(modelFile);

    // read marker data from file
    MarkerData markerData(trcFile);

    // prepare marker tasks
    vector<InverseKinematics::MarkerTask> markerTasks;
    vector<string> observationOrder;
    InverseKinematics::createMarkerTasksFromMarkerData(
            model, markerData, markerTasks, observationOrder);

    // initialize marker reconstruction
    MarkerReconstruction mmr(model, markerTasks);
    auto logger = mmr.initializeLogger();

    // loop through marker frames
    for (int i = 0; i < markerData.getNumFrames(); ++i) {
        // get frame data
        auto ikFrame = InverseKinematics::getFrameFromMarkerData(
                i, markerData, observationOrder, false);
        auto& originalMarkers = ikFrame.markerObservations;
        auto& t = ikFrame.t;

        const auto missingMarkers = // NOTE overload on const-ness to return a
                                    // copy of the reconstructed markers
                simulateMissingMarkers(t, originalMarkers, observationOrder,
                                       missingMarkerLabels, duration, t0);

        // try to initilize the MMR with a valid frame
        if (!mmr.initState(missingMarkers)) continue;

        // perform marker reconstruction
        auto reconstructedMarkers = mmr.solve(missingMarkers);

        // record
        logger.appendRow(t, reconstructedMarkers);
    }

    // store results
    STOFileAdapter::write(logger.flatten(),
                          subjectDir + "real_time/marker_reconstruction/"
                                       "reconstructed_markers.sto");
}

int main(int argc, char* argv[]) {
    try {
        run();
    } catch (exception& e) {
        cout << e.what() << endl;
        return -1;
    }
    return 0;
}
