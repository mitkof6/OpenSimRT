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
#include "NGIMUData.h"

using namespace OpenSimRT;
using namespace SimTK;

SimTK::Quaternion NGIMUData::getQuaternion() const {
    return this->quaternion.q;
}

Vector NGIMUData::asVector() const {
    Vector v(this->size());
    int i = 0;
    v(i, 4) = Vector(this->quaternion.q);
    i += 4;
    v(i, 3) = Vector(this->sensors.acceleration);
    i += 3;
    v(i, 3) = Vector(this->sensors.gyroscope);
    i += 3;
    v(i, 3) = Vector(this->sensors.magnetometer);
    i += 3;
    v(i, 1) = Vector(this->sensors.barometer);
    i += 1;
    v(i, 3) = Vector(this->linear.acceleration);
    i += 3;
    v(i, 1) = Vector(this->altitude.measurement);
    return v;
}

void NGIMUData::fromVector(const Vector& v) {
    int i = 0;
    this->quaternion.q = SimTK::Quaternion(v[0], v[1], v[2], v[3]);
    this->sensors.acceleration = Vec3(&v[4]);
    this->sensors.gyroscope = Vec3(&v[7]);
    this->sensors.magnetometer = Vec3(&v[10]);
    this->sensors.barometer = Vec1(&v[13]);
    this->linear.acceleration = Vec3(&v[14]);
    this->altitude.measurement = Vec1(&v[17]);
}

NGIMUData::NGIMUPack NGIMUData::getAsPack() const {
    std::vector<std::pair<double, SimTK::Vector>> res{
            std::make_pair(this->quaternion.timeStamp,
                           Vector(this->quaternion.q)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.acceleration)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.gyroscope)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.magnetometer)),
            std::make_pair(this->sensors.timeStamp,
                           Vector(this->sensors.barometer)),
            std::make_pair(this->linear.timeStamp,
                           Vector(this->linear.acceleration)),
            std::make_pair(this->altitude.timeStamp,
                           Vector(this->altitude.measurement))};
    return res;
}

void NGIMUData::setFromPack(const NGIMUPack& pack) {
    const auto& v = pack[0].second;
    this->quaternion.q = SimTK::Quaternion(v[0], v[1], v[2], v[3]);
    this->sensors.acceleration = Vec3(&pack[1].second[0]);
    this->sensors.gyroscope = Vec3(&pack[2].second[0]);
    this->sensors.magnetometer = Vec3(&pack[3].second[0]);
    this->sensors.barometer = Vec1(&pack[4].second[0]);
    this->linear.acceleration = Vec3(&pack[5].second[0]);
    this->altitude.measurement = Vec1(&pack[6].second[0]);

    this->quaternion.timeStamp = pack[0].first;
    this->sensors.timeStamp = pack[1].first;
    this->linear.timeStamp = pack[5].first;
    this->altitude.timeStamp = pack[6].first;
}
