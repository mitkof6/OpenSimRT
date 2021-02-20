#include "MarkerReconstruction.h"

#include "OpenSimUtils.h"

using namespace std;
using namespace OpenSim;
using namespace OpenSimRT;
using namespace SimTK;

// MarkerReconstruction constructor
MarkerReconstruction::MarkerReconstruction(
        const OpenSim::Model& otherModel,
        const vector<InverseKinematics::MarkerTask>& markerTasks)
        : model(*otherModel.clone()), isInitialized(false) {
    // map markers to corresponding body segments
    for (int i = 0; i < markerTasks.size(); ++i) {
        const auto& markerName = markerTasks[i].name;
        const auto& marker = model.getMarkerSet().get(markerName);
        markersPerBodyMap.emplace(marker.getParentFrameName(), markerName);

        // get marker observation order from tasks.
        observationOrder.push_back(markerName);
    }
    // create table with distances between markers in same body
    for (int i = 0; i < markerTasks.size(); ++i) {
        const auto& iMarkerName = markerTasks[i].name;
        const auto& iMarker = model.getMarkerSet().get(iMarkerName);

        const auto& markersInBody =
                markersPerBodyMap.equal_range(iMarker.getParentFrameName());

        // for every marker in body calculate the distance between marker i,j
        for (auto itr = markersInBody.first; itr != markersInBody.second;
             ++itr) {
            const auto& jMarkerName = itr->second;
            const auto& jMarker = model.getMarkerSet().get(jMarkerName);

            Vec3 dij = jMarker.get_location() - iMarker.get_location();
            markerDistanceTable[iMarkerName][jMarkerName] = dij;
        }
    }
}

bool MarkerReconstruction::isValidFrame(
        const SimTK::Array_<SimTK::Vec3>& markerObservations) {
    // check for missing markers
    for (const auto& m : markerObservations) {
        if (m == Vec3(0) || !m.isFinite()) { return false; }
    }
    return true;
}

bool MarkerReconstruction::initState(
        const SimTK::Array_<SimTK::Vec3>& markerObservations) {
    if (!isInitialized) {
        if (isValidFrame(markerObservations)) {
            // assign the valid frame to the initial state of the previous known
            // observations.
            previousObservations = markerObservations;
            isInitialized = true;
        }
    }
    return isInitialized;
}

void MarkerReconstruction::solve(Array_<Vec3>& currentObservations) {
    for (unsigned int i = 0; i < currentObservations.size(); i++) {
        if (!currentObservations[i].isFinite()) {
            // find the closest two markers of the missing marker.
            const auto& mMarkerName = observationOrder[i];
            auto indices =
                    findClosestMarkers(mMarkerName, currentObservations, 3);

            if (indices.empty()) { // no marker found
                currentObservations[i] = previousObservations[i];

            } else if (indices.size() == 1) { // one marker found
                int id1 = indices[0];
                auto d1x = previousObservations[id1] - previousObservations[i];
                currentObservations[i] = currentObservations[id1] - d1x;

            } else if (indices.size() == 2) { // two markers found
                int id1 = indices[0];
                int id2 = indices[1];

                // distance vectors between the missing and closest markers
                auto d1x = previousObservations[id1] - previousObservations[i];
                auto d2x = previousObservations[id2] - previousObservations[i];
                // const auto& d1x =
                // markerDistanceTable[observationOrder[i]][observationOrder[id1]];
                // const auto& d2x =
                // markerDistanceTable[observationOrder[i]][observationOrder[id2]];

                // estimate of missing marker based on distance.
                auto x_tilde = ((currentObservations[id1] - d1x) +
                                (currentObservations[id2] - d2x)) /
                               2.0;

                // sphere objects
                Sphere c1{currentObservations[id1], d1x.norm()};
                Sphere c2{currentObservations[id2], d2x.norm()};

                // find intersection of the two spheres.
                auto* circle = sphereSphereIntersection(c1, c2);

                // reconstructed point is the point on the circle closest to
                // x_tilde.
                if (circle) {
                    // projection on the plane of the circle
                    currentObservations[i] =
                            closestPointToCircle(x_tilde, circle);

                } else { // spheres do not intersect.
                    currentObservations[i] = x_tilde;
                }
                delete circle;

            } else { // find best tranformation that fits the data

                const int& id1 = indices[0];
                const int& id2 = indices[1];
                const int& id3 = indices[2];

                Matrix A(3, 3);
                Matrix B(3, 3);

                // marker coordinates as columns in matrices
                for (int j = 0; j < 3; ++j) {
                    A.updCol(j) =
                            Vector(3, &previousObservations[indices[j]][0]);
                }
                for (int j = 0; j < 3; ++j) {
                    B.updCol(j) =
                            Vector(3, &currentObservations[indices[j]][0]);
                }

                // find centroid and substract from columns in A and B
                const auto centroid_A = A.rowSum() / A.ncol();
                const auto centroid_B = B.rowSum() / B.ncol();

                for (int j = 0; j < A.ncol(); ++j) {
                    A.updCol(j) -= centroid_A;
                }
                for (int j = 0; j < B.ncol(); ++j) {
                    B.updCol(j) -= centroid_B;
                }

                // solve SVD for H = A * B**T --> [U, S, V**T] = SVD(H)
                Matrix rightVectors;
                Matrix leftVectors;
                Vector singularValues;

                FactorSVD svd(A * (~B));
                svd.getSingularValuesAndVectors(singularValues, leftVectors,
                                                rightVectors);

                // rotation matrix R = V * U**T
                auto r = (~rightVectors) * (~leftVectors);
                auto R = Mat33(r[0][0], r[0][1], r[0][2], r[1][0], r[1][1],
                               r[1][2], r[2][0], r[2][1], r[2][2]);

                // address reflexion case
                if (det(R) < 0) {
                    rightVectors[2] *= -1;
                    r = (~rightVectors) * (~leftVectors);
                    R = Mat33(r[0][0], r[0][1], r[0][2], r[1][0], r[1][1],
                              r[1][2], r[2][0], r[2][1], r[2][2]);
                }

                // translation vector
                auto t = centroid_B - Matrix(R) * centroid_A;

                // transform missing marker to compute a current estimate
                auto estimate = Matrix(R) * Vector(previousObservations[i]) + t;
                currentObservations[i] =
                        Vec3(estimate[0], estimate[1], estimate[2]);
            }
        }
    }
    // update previous observations
    previousObservations = currentObservations;
}

SimTK::Array_<SimTK::Vec3> MarkerReconstruction::solve(
        const SimTK::Array_<SimTK::Vec3>& currentObservations) {
    auto reconstructedObservations(currentObservations);
    solve(reconstructedObservations);
    return reconstructedObservations;
}

MarkerReconstruction::Circle*
MarkerReconstruction::sphereSphereIntersection(const Sphere& c1,
                                               const Sphere& c2) {
    double d = (c1.origin - c2.origin).norm();        // distance of two origins
    Vec3 d_hat = (c2.origin - c1.origin).normalize(); // unit vector

    // check for solvability.
    if (d > (c1.radius + c2.radius) || (d < abs(c1.radius - c2.radius))) {
        return nullptr;
    }

    // determine the distance from c1.origin to point p.
    double a = (pow(c1.radius, 2) - pow(c2.radius, 2) + pow(d, 2)) / (2.0 * d);

    // determine the coordinates of point p.
    auto p = c1.origin + a * d_hat;

    // determine the distance from point p to either of the intersection points.
    double h = sqrt((c1.radius * c1.radius) - (a * a));
    // determine the intersection points.
    return new Circle{p, d_hat, h};
}

Vec3 MarkerReconstruction::closestPointToCircle(const Vec3& vec,
                                                const Circle* c) {
    auto dist = dot(vec - c->origin, c->normal);
    auto vec_prime = vec - dist * c->normal;

    // shortest distance between a point and a circle in 2D.
    auto n = (vec_prime - c->origin).normalize();
    return c->origin + n * c->radius;
};

vector<int> MarkerReconstruction::findClosestMarkers(
        const string& mMarkerName, const Array_<Vec3>& currentObservations,
        int numMarkers) {
    // info of missing marker
    const auto& mMarker = model.getMarkerSet().get(mMarkerName);
    const auto& bodyName = mMarker.getParentFrameName();
    const int idx = distance(observationOrder.cbegin(),
                             find(observationOrder.cbegin(),
                                  observationOrder.cend(), mMarkerName));

    // marker list per body, ignoring missing markers
    vector<string> markersInBody;
    for (auto itr = markersPerBodyMap.equal_range(bodyName).first;
         itr != markersPerBodyMap.equal_range(bodyName).second; ++itr) {
        const int i = distance(observationOrder.cbegin(),
                               find(observationOrder.cbegin(),
                                    observationOrder.cend(), itr->second));

        if (currentObservations[i].isFinite()) {
            markersInBody.push_back(itr->second);
        }
    }

    // compare function based on marker distance
    auto comp = [&](const string& iMarker, const string& jMarker) -> bool {
        const auto& d1 = markerDistanceTable[mMarkerName][iMarker];
        const auto& d2 = markerDistanceTable[mMarkerName][jMarker];
        return d1.norm() < d2.norm();
    };

    // sort markers in body based on distance
    sort(markersInBody.begin(), markersInBody.end(), comp);

    // return the N closest marker IDs
    vector<int> output;
    numMarkers = min((int) markersInBody.size(), numMarkers);
    for (auto itr = markersInBody.cbegin();
         itr != markersInBody.cbegin() + numMarkers; ++itr) {
        output.push_back(distance(observationOrder.cbegin(),
                                  find(observationOrder.cbegin(),
                                       observationOrder.cend(), *itr)));
    }
    return output;
}

TimeSeriesTable_<SimTK::Vec3> MarkerReconstruction::initializeLogger() {
    TimeSeriesTable_<SimTK::Vec3> table;
    table.setColumnLabels(observationOrder);
    return table;
}
