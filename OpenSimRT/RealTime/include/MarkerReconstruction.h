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
 * Reconstruct missing markers based on their previously known positions and the
 * positions of their closest markers in the same body in the current frame. The
 * basic notion behind this method is that the distance between markers in the
 * same body does not change between frames.
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
        double radius;
        SimTK::Vec3 origin;
        SimTK::Vec3 normal;
    };

    // sphere representtion
    struct Sphere {
        double radius;
        SimTK::Vec3 origin;
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
    /**
     * Overloaded function of the marker reconstruction method. Case where no
     * valid markers exist in the same body. Returns the previously known
     * positions.
     */
    void reconstructionMethod(SimTK::Array_<SimTK::Vec3>& currentObservations,
                              const int& i);
    /**
     * Overloaded function of the marker reconstruction method. Case where one
     * valid markers exist in the same body. Missing marker is estimated
     * based on the distance vector between of the two markers in the previous
     * valid frame and the current position of the known marker.
     */
    void reconstructionMethod(SimTK::Array_<SimTK::Vec3>& currentObservations,
                              const int& i, const int& id1);
    /**
     * Overloaded function of the marker reconstruction method. Case where two
     * valid markers exist in the same body. Missing marker is estimated
     * based on the distance vectors of the two known markers and the missing
     * one in the previous valid frame and the current position of the known
     * markers. The position of the missing marker lies in a circle found by the
     * intersection of two spheres with origin the known marker positions and
     * radious the distance vectors.
     */
    void reconstructionMethod(SimTK::Array_<SimTK::Vec3>& currentObservations,
                              const int& i, const int& id1, const int& id2);
    /**
     * Overloaded function of the marker reconstruction method. Case where more
     * than two valid markers exist in the same body. Missing marker is
     * estimated after a applying a rigid transformation that best transforms
     * the known markers from the previous frame to the current frame.
     */
    void reconstructionMethod(SimTK::Array_<SimTK::Vec3>& currentObservations,
                              const int& i, const std::vector<int>& indices);
    OpenSim::Model model;
    DistanceTable markerDistanceTable;
    std::multimap<std::string, std::string> markersPerBodyMap;
    std::vector<std::string> observationOrder;
    SimTK::Array_<SimTK::Vec3> previousObservations;
    bool isInitialized;
};
} // namespace OpenSimRT
