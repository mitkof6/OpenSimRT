/**
 * @file SimulationUtils.h
 *
 * \brief Common utilities shared by the different modules.
 *
 * @author Dimitar Stanev <dimitar.stanev@epfl.ch>
 */
#ifndef SIMULATION_UTILS_H
#define SIMULATION_UTILS_H

#include <string>
#include <deque>
#include <limits>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <functional>
#define _USE_MATH_DEFINES
#include <math.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <simbody/internal/Visualizer_InputListener.h>
#include "internal/RealTimeExports.h"

 /**
  * Converts an OpenSim::Array to std container <U> (e.g., vector<double>).
  */
template<typename T, typename U> void osimToStd(const T& srcArray, U& dstVector) {
    dstVector.clear();
    int size = srcArray.getSize();
    dstVector.resize(size);
    for (int i = 0; i < size; ++i) {
        dstVector.at(i) = srcArray.get(i);
    }
}

/**
 * Converts a Simbody (e.g., Vector) to std container (e.g., vector<double>).
 */
template<typename T, typename U> void simtkToStd(const T& srcArray, U& dstVector) {
    dstVector.clear();
    int size = srcArray.size();
    dstVector.resize(size);
    for (int i = 0; i < size; ++i) {
        dstVector.at(i) = srcArray.get(i);
    }
}

/**
 * Converts <T> to string and with defined precision (in case of number).
 */
template<typename T>
std::string toString(const T& value,
                     int precision = std::numeric_limits<int>::infinity()) {
    std::ostringstream oss;
    if (precision != std::numeric_limits<int>::infinity()) {
        oss << std::setprecision(precision);
    }
    oss << value;
    return oss.str();
}

/**
 * Separates (delimiter) the values of the std container into a single line
 * string.
 */
template<typename T>
std::string dump(const T& vec, std::string delimiter,
                 int precision = std::numeric_limits<int>::infinity()) {
    std::string row = toString(vec.at(0));
    for (int i = 1; i < vec.size(); ++i) {
        row += delimiter;
        row += toString(vec.at(i));
    }
    return row;
}

// used as macro in order to insert __FILE__ and __LINE__
#define THROW_EXCEPTION(msg) throw FileLineException(msg, __FILE__, __LINE__)

// ensure that i is within a range to avoid segmentation fault
#define ENSURE_BOUNDS(i, min, max)	                                	\
    (i >= min && i <= max ) ? i : THROW_EXCEPTION("ENSURE_BOUNDS failed")

// start measuring time
#define START_CHRONO()							        \
    std::chrono::high_resolution_clock::time_point t1;			        \
    t1 = std::chrono::high_resolution_clock::now();

// end measuring time
#define END_CHRONO()					            		\
    std::chrono::high_resolution_clock::time_point t2;			        \
    t2 = std::chrono::high_resolution_clock::now();     			\
    std::cout << "Elapsed time: "			        		\
    << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()   \
    << "ms" << std::endl;

// Generates a unique identifier
RealTime_API int generateUID();

/**
 * Extract model's coordinate names.
 */
RealTime_API std::vector<std::string> getCoordinateNames(const OpenSim::Model& model);

/**
 * Extract model's muscle names.
 */
RealTime_API std::vector<std::string> getMuscleNames(const OpenSim::Model& model);

/**
 * Extract model's actuator names.
 */
RealTime_API std::vector<std::string> getActuatorNames(const OpenSim::Model& model);

/**
 * \brief An exception that prints the file and line number along with
 * the message.
 */
class FileLineException : public std::exception {
    std::string msg;
public:
    FileLineException(const std::string &arg, const char* file, int line)
        : std::exception() {
        std::ostringstream o;
        o << file << ":" << line << ": " << arg;
        msg = o.str();
    }
    ~FileLineException() throw() {};
    const char* what() const throw() { return msg.c_str(); }
};

/**
 * \brief A simple csv logger.
 */
class RealTime_API CSVLogger {
public:
    typedef std::vector<double> row_t;
    typedef std::vector<std::string> columns_t;
    columns_t columns;
    std::vector<row_t> data;
    std::string delimiter = ",";
public:
    CSVLogger(const columns_t& columns);
    void addRow(const row_t& row);
    void addRow(double t, const SimTK::Vector& vec);
    void exportToFile(std::string file);
};

/**
 * \brief Displays the FPS and duration through the Simbody visualizer.
 *
 * @code
 *    // setup visualizer
 *    model.setUseVisualizer(true);
 *    model.initSystem();
 *    auto& visualizer = model.updVisualizer().updSimbodyVisualizer();
 *    visualizer.setShowFrameRate(false);
 *    visualizer.setShutdownWhenDestructed(true);
 *    visualizer.setMode(Visualizer::Mode::Sampling);
 *    visualizer.setDesiredBufferLengthInSec(5);
 *    visualizer.setDesiredFrameRate(60);
 *    FPSDecorator* fps = new FPSDecorator();
 *    visualizer.addDecorationGenerator(fps);
 *    ...
 *    // measure execution time and update visualizer
 *    fps->measureFPS();
 *    visualizer.report(id.state);
 * @endcode
 */
class RealTime_API FPSDecorator : public SimTK::DecorationGenerator {
public:
    std::string text;
public:
    FPSDecorator();
    void generateDecorations(const SimTK::State& state,
                             SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
    void measureFPS();
};

/**
 * \brief Visualize a force that is applied on a body.
 */
class RealTime_API ForceDecorator : public SimTK::DecorationGenerator {
public:
    SimTK::Vec3 color;
    SimTK::Vec3 point;
    SimTK::Vec3 force;
    double scaleFactor;
    int lineThikness;
public:
    ForceDecorator(SimTK::Vec3 color, double scaleFactor, int lineThikness);
    void update(SimTK::Vec3 point, SimTK::Vec3 force);
    void generateDecorations(const SimTK::State& state,
                             SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
};

/**
 * \brief Simple model visualizer. 
 *
 * If ESC key is pressed shouldTerminate = true. This can be used to terminate a
 * simulation (e.g., infinite loop).
 */
class RealTime_API BasicModelVisualizer {
public:
    OpenSim::Model model;
    SimTK::State state;
    FPSDecorator* fps;
    SimTK::Visualizer* visualizer;
    SimTK::Visualizer::InputSilo* silo;
    bool shouldTerminate;
public:
    BasicModelVisualizer(std::string modelFile);
    void update(const SimTK::Vector& q,
		const SimTK::Vector& muscleActivations = SimTK::Vector());
};

#if __linux__
/**
 * \brief Uses gnuplot to visualize time series of limited history.
 */
template<int N, int M> class GNUPlot {
public:
    int uid;
    FILE* gnuplot;
    int current;
    bool startOver;
    SimTK::Matrix data;
public:
    GNUPlot(bool persist = false) {
        // generate unique identifier
        uid = generateUID();
        // initialize variables
        current = 0;
        startOver = false;
        data = SimTK::Matrix(M, N, 0.0);
        // open gnuplot pipe
        if (persist) {
            gnuplot = popen("gnuplot --persist ", "w");
        } else {
            gnuplot = popen("gnuplot", "w");
        }
    }

    ~GNUPlot() {
        pclose(gnuplot);
        remove(toString(uid).c_str()); // remove unique data file
    }

    void add(SimTK::Vector rowVector) {
        if (rowVector.size() != M) {
            THROW_EXCEPTION("row vector dimensionality mismatch " +
                            toString(rowVector.size()) + " != " + toString(M));
        }
        // update history
        data(current) = rowVector;
        // check buffer size
        current++;
        if (current == N) {
            current = 0;
        }
    }

    void visualize(std::vector<int> column,
                   std::vector<std::string> style,
                   std::vector<std::string> title) {
        // avoid calling gnuplot with 0 or 1 points
        if (!startOver && current < 2) return;
        // write data to a binary file
        std::string dataFileName = toString(uid);
        std::ofstream fData(dataFileName, std::ios::out | std::ios::binary);
        fData.write((char*) &data[0][0],
                    data.ncol() * data.nrow() * sizeof(SimTK::Real));
        fData.close();
        // send command to gnuplot
        int record = !startOver ? current : N;
        std::string command = "set autoscale\n";
        for (int i = 0; i < column.size(); ++i) {
            std::string prefix;
            if (i != 0) {
                prefix = ", ";
            } else {
                prefix = "plot ";
            }
            command += prefix;
            command += "'" + dataFileName + "' ";
            command += "binary record=" + toString(record) + " ";
            command += std::string("format=") + "'%" + toString(M) + "double' ";
            command += "using 1:" + toString(column[i] + 1) + " ";
            command += "with " + style[i] + " ";
            command += "title '" + title[i] + "' ";
        }
        fprintf(gnuplot, "%s\n", command.c_str());
        fflush(gnuplot);
    }
};
#endif

/**
 * \brief A thread safe circular buffer.
 */
template<int history, typename T> class CircularBuffer {
private:
    int current;
    bool startOver;
    std::vector<T> buffer;
    std::mutex monitor;
    std::condition_variable bufferNotEmpty;
public:
    CircularBuffer() {
        current = 0;
        startOver = false;
        buffer.resize(history);
    }

    bool notEmpty(int M) {
        if (startOver && history >= M) {
            return true;
        } else if (!startOver && current >= M) {
            return true;
        } else {
            return false;
        }
    }

    void add(const T& value) {
        {
            // lock
            std::lock_guard<std::mutex> lock(monitor);
            // update buffer
            buffer[current] = value;
            current++;
            if (current == history) {
                current = 0;
                startOver = true;
            }
        }
        // notify after unlocking
        bufferNotEmpty.notify_one();
    }

    std::vector<T> get(int M, bool reverse = false) {
        if (M <= 0 || M > history) {
            THROW_EXCEPTION("M should be between [1, history]");
        }
        // lock
        std::unique_lock<std::mutex> lock(monitor);
        // check if data are available to proceed
        auto predicate = std::bind(&CircularBuffer::notEmpty, this, M);
        bufferNotEmpty.wait(lock, predicate);
        // if not empty get data
        std::vector<T> result;
        result.resize(M);
        int index = current - 1;
        for (int i = 0; i < M; ++i) { // order of execution matters
            if (index < 0) {
                index = history - 1;
            }
            result[i] = buffer[index];
            index--;
        }
        if (reverse) {
            std::reverse(result.begin(), result.end());
        }
        return result;
    }
};

#endif
