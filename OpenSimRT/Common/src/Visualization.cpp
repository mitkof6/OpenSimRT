#include "Visualization.h"

#include "Utils.h"

#include <OpenSim/Simulation/Model/Muscle.h>

using namespace std;
using namespace chrono;
using namespace SimTK;
using namespace OpenSimRT;

/******************************************************************************/

FPSDecorator::FPSDecorator() : text("") {}

void FPSDecorator::generateDecorations(const State& state,
                                       Array_<DecorativeGeometry>& geometry) {
    DecorativeText info;
    info.setIsScreenText(true);
    info.setText(text);
    geometry.push_back(info);
}

milliseconds FPSDecorator::calculateLoopDelay() {
    static int counter = 0;
    static high_resolution_clock::time_point previousTime =
            high_resolution_clock::now();
    counter++;
    auto currentTime = high_resolution_clock::now();
    auto duration = duration_cast<milliseconds>(currentTime - previousTime);
    if (duration > milliseconds(1000)) {
        previousTime = currentTime;
        text = "FPS: " + to_string(counter) +
               " | Delay: " + toString(1000.0 / counter, 2) + "ms";
        counter = 0;
    }
    return duration;
}

/******************************************************************************/

ForceDecorator::ForceDecorator(Vec3 color, double scaleFactor, int lineThikness)
        : color(color), scaleFactor(scaleFactor), lineThikness(lineThikness) {}

void ForceDecorator::update(SimTK::Vec3 point, SimTK::Vec3 force) {
    this->point = point;
    this->force = force;
}

void ForceDecorator::generateDecorations(const State& state,
                                         Array_<DecorativeGeometry>& geometry) {
    DecorativeLine line(point, point + scaleFactor * force);
    line.setColor(color);
    line.setLineThickness(lineThikness);
    geometry.push_back(line);
}

/******************************************************************************/

BasicModelVisualizer::BasicModelVisualizer(const OpenSim::Model& otherModel)
        : model(*otherModel.clone()), shouldTerminate(false) {
#ifndef CONTINUOUS_INTEGRATION
    model.setUseVisualizer(true);
#endif

    state = model.initSystem();

#ifndef CONTINUOUS_INTEGRATION
    visualizer = &model.updVisualizer().updSimbodyVisualizer();
    silo = &model.updVisualizer().updInputSilo();
    visualizer->setShowFrameRate(false);
    visualizer->setShutdownWhenDestructed(true);
    visualizer->setMode(Visualizer::Mode::Sampling);
    visualizer->setDesiredBufferLengthInSec(5);
    visualizer->setDesiredFrameRate(60);
    fps = new FPSDecorator();
    visualizer->addDecorationGenerator(fps);
#endif
}

void BasicModelVisualizer::update(const Vector& q,
                                  const Vector& muscleActivations) {
#ifndef CONTINUOUS_INTEGRATION
    fps->calculateLoopDelay();
#endif
    // kinematics
    state.updQ() = q;
    // muscle activations
    // TODO handle path actuators
    if (muscleActivations.size() == model.getMuscles().getSize()) {
        for (int i = 0; i < model.getMuscles().getSize(); ++i) {
            // model.updMuscles().get(i).setActivation(state, muscleActivations[i]);
            model.updMuscles().get(i).getGeometryPath()
                .setColor(state, Vec3(muscleActivations[i], 0, 1 - muscleActivations[i]));
        }
    }
#ifndef CONTINUOUS_INTEGRATION
    visualizer->report(state);
    // terminate if ESC key is pressed
    unsigned key, modifiers;
    if (silo->takeKeyHit(key, modifiers)) {
        if (key == Visualizer::InputListener::KeyEsc) {
            shouldTerminate = true;
        }
    }
#endif
}

void BasicModelVisualizer::addDecorationGenerator(DecorationGenerator* generator) {
#ifndef CONTINUOUS_INTEGRATION
    visualizer->addDecorationGenerator(generator);
#endif
}

/******************************************************************************/
