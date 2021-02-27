#include "NGIMUInputFromFileDriver.h"

#include "Exception.h"

#include <iostream>

using namespace OpenSimRT;
using namespace SimTK;

NGIMUInputFromFileDriver::NGIMUInputFromFileDriver(const std::string& fileName,
                                                   const double& sendRate)
        : table(fileName), rate(sendRate) {}

NGIMUInputFromFileDriver::~NGIMUInputFromFileDriver() { t.join(); }

void NGIMUInputFromFileDriver::startListening() {
    static auto f = [&]() {
        try {
            int i = 0;
            while (true) {
                {
                    std::lock_guard<std::mutex> lock(mu);
                    time = table.getIndependentColumn()[i];
                    frame = table.getMatrix()[i];
                    newRow = true;
                }
                cond.notify_one();

                // increase counter
                if (i < table.getNumRows())
                    ++i;
                else
                    THROW_EXCEPTION("End of file. Stop streaming.");

                std::this_thread::sleep_for(std::chrono::milliseconds(
                        static_cast<int>(1 / rate * 1000)));
            }
        } catch (std::exception& e) {
            std::cout << e.what() << std::endl;
            exc_ptr = std::current_exception();
        }
    };
    t = std::thread(f);
}

bool NGIMUInputFromFileDriver::shouldTerminate() {
    return !(exc_ptr == nullptr);
}

NGIMUInputFromFileDriver::IMUDataList
NGIMUInputFromFileDriver::fromVector(const Vector& v) const {
    IMUDataList list;
    NGIMUData data;
    int n = NGIMUData::size();
    for (int i = 0; i < v.size(); i += n) {
        data.fromVector(v(i, n));
        list.push_back(data);
    }
    return list;
}

NGIMUInputFromFileDriver::IMUDataList NGIMUInputFromFileDriver::getData() const {
    std::unique_lock<std::mutex> lock(mu);
    cond.wait(lock, [&]() { return (newRow == true) || (exc_ptr != nullptr); });
    newRow = false;
    return fromVector(frame.getAsVector());
}

std::pair<double, std::vector<NGIMUData>> NGIMUInputFromFileDriver::getFrame() {
    auto temp = getFrameAsVector();
    return std::make_pair(temp.first, fromVector(temp.second));
}

std::pair<double, Vector> NGIMUInputFromFileDriver::getFrameAsVector() const {
    std::unique_lock<std::mutex> lock(mu);
    cond.wait(lock, [&]() { return (newRow == true) || (exc_ptr != nullptr); });
    newRow = false;
    return std::make_pair(time, frame.getAsVector());
}
