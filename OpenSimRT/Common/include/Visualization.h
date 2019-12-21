#ifndef VISUALIZATION_H
#define VISUALZIATION_H

#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <simbody/internal/Visualizer_InputListener.h>
#include "internal/CommonExports.h"

namespace OpenSimRT {

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
class Common_API FPSDecorator : public SimTK::DecorationGenerator {
 public:
    FPSDecorator();
    void generateDecorations(const SimTK::State& state,
                             SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
    void measureFPS();
 private:
    std::string text;
};

/**
 * \brief Visualize a force that is applied on a body.
 */
class Common_API ForceDecorator : public SimTK::DecorationGenerator {
 public:
    ForceDecorator(SimTK::Vec3 color, double scaleFactor, int lineThikness);
    void update(SimTK::Vec3 point, SimTK::Vec3 force);
    void generateDecorations(const SimTK::State& state,
                             SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
 private:
    SimTK::Vec3 color;
    SimTK::Vec3 point;
    SimTK::Vec3 force;
    double scaleFactor;
    int lineThikness;
};

/**
 * \brief Simple model visualizer.
 *
 * If ESC key is pressed shouldTerminate = true. This can be used to terminate a
 * simulation (e.g., infinite loop).
 */
class Common_API BasicModelVisualizer {
 public:
    BasicModelVisualizer(std::string modelFile);
    void update(const SimTK::Vector& q,
                const SimTK::Vector& muscleActivations = SimTK::Vector());
 private:
    OpenSim::Model model;
    SimTK::State state;
    FPSDecorator* fps;
    SimTK::Visualizer* visualizer;
    SimTK::Visualizer::InputSilo* silo;
    bool shouldTerminate;
};

} // namespace OpenSimRT

#endif
