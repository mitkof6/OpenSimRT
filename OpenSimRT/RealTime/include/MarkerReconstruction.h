/**
 * @file MarkerReconstruction.h
 *
 * Real-time missing marker reconstruction method.
 *
 * @author Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once
#include "InverseKinematics.h"
#include "internal/RealTimeExports.h"

#include <Common/TimeSeriesTable.h>
#include <SimTKcommon.h>

namespace OpenSimRT {

/**
 * Reconstruct missing markers based on previous valid frame
 * and the closest markers in current frame. The basic notion behind this
 * method is that the distance between close markers does not change between
 * frames.
 *
 * Modified implementation of Aristidou et.al.
 * "Real-time estimation of missing markers in human motion capture".
 * https://ieeexplore.ieee.org/abstract/document/4535545
 */
class RealTime_API MarkerReconstruction {
 public:
    /**
     * Construct the Missing Marker Module.
     */
    MarkerReconstruction(
            const OpenSim::Model& model,
            const std::vector<InverseKinematics::MarkerTask>& markerTasks);

    /**
     * Initialize internal state. Returns true if it's initialized when input is
     * valid, else returns false.
     */
    bool initState(const SimTK::Array_<SimTK::Vec3>& markerObservations);

    /**
     * Fill the missing marker by passing a reference to the current marker
     * observations.
     */
    void solve(SimTK::Array_<SimTK::Vec3>& currentObservations);

    /**
     * Overloaded function that returns a copy of the reconstructed markers.
     */
    SimTK::Array_<SimTK::Vec3>
    solve(const SimTK::Array_<SimTK::Vec3>& currentObservations);

    /**
     * Initialize logger.
     */
    OpenSim::TimeSeriesTable_<SimTK::Vec3> initializeLogger();

 private:
    // A table alias that keeps track of the distances among markers in the same
    // body based on their location defined in the osim model.
    typedef std::map<std::string, std::map<std::string, SimTK::Vec3>>
            DistanceTable;

    // circle representation
    struct Circle {
        SimTK::Vec3 origin;
        SimTK::Vec3 normal;
        double radius;
    };

    // sphere representtion
    struct Sphere {
        SimTK::Vec3 origin;
        double radius;
    };

    /**
     * Determine if an input frame is valid.
     */
    bool isValidFrame(const SimTK::Array_<SimTK::Vec3>& markerObservations);

    /**
     *  Find intersection of two spheres, i.e. a circle in the 3D space.
     */
    Circle* sphereSphereIntersection(const Sphere& c1, const Sphere& c2);

    /**
     * Given a point, find its closest point in the circumference of a
     * given circle lying in the 3D space.
     */
    SimTK::Vec3 closestPointToCircle(const SimTK::Vec3& vec, const Circle* c);

    /**
     * Find the closests N markers (indexes) to the missing marker.
     *
     * @param [mMarkerName]: name of the missing marker
     * @param [currentObservations] - Array with positions of the markers in
     * current frame.
     * @param [numMarkers] - N closest markers
     *
     * @return [id1] - reference to the index of the closest marker.
     * @return [id2] - reference to the index of the second closest marker.
     */
    std::vector<int>
    findClosestMarkers(const std::string& mMarkerName,
                       const SimTK::Array_<SimTK::Vec3>& currentObservations,
                       int numMarkers);

    OpenSim::Model model;
    DistanceTable markerDistanceTable;
    std::multimap<std::string, std::string> markersPerBodyMap;
    std::vector<std::string> observationOrder;
    SimTK::Array_<SimTK::Vec3> previousObservations;
    bool isInitialized;
};
} // namespace OpenSimRT
