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
 *
 * @file Visualization.h
 *
 * \brief OpenSim basic visualization primitives.
 *
 * @author Dimitar Stanev <jimstanev@gmail.com>
 * contribution: Filip Konstantinos <filip.k@ece.upatras.gr>
 */
#pragma once

#include "internal/CommonExports.h"
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/Model/ModelVisualizer.h>
#include <SimTKcommon/internal/DecorationGenerator.h>
#include <simbody/internal/Visualizer_InputListener.h>

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
    void generateDecorations(
            const SimTK::State& state,
            SimTK::Array_<SimTK::DecorativeGeometry>& geometry) override;
    std::chrono::milliseconds calculateLoopDelay();

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
    void generateDecorations(
            const SimTK::State& state,
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
    BasicModelVisualizer(const OpenSim::Model& model);
    // Update visualizer state.
    void update(const SimTK::Vector& q,
                const SimTK::Vector& muscleActivations = SimTK::Vector());
    void updateReactionForceDecorator(
            const SimTK::Vector_<SimTK::SpatialVec>& reactionWrench,
            const std::string& reactionOnBody,
            ForceDecorator* reactionForceDecorator);
    // Add decoration generator to the visualizer (take ownership of the
    // memory).
    void addDecorationGenerator(SimTK::DecorationGenerator* generator);
    void expressPositionInGround(const std::string& fromBodyName,
                                 const SimTK::Vec3& fromBodyPoint,
                                 SimTK::Vec3& toBodyPoint);
    void expressPositionInAnotherFrame(const std::string& fromBodyName,
                                       const SimTK::Vec3& fromBodyPoint,
                                       const std::string& toBodyName,
                                       SimTK::Vec3& toBodyPoint);

 private:
    OpenSim::Model model;
    SimTK::State state;
    SimTK::ReferencePtr<FPSDecorator> fps;
    SimTK::ReferencePtr<SimTK::Visualizer> visualizer;
    SimTK::ReferencePtr<SimTK::Visualizer::InputSilo> silo;
    bool shouldTerminate;

    enum class MenuID { SIMULATION }; //// TODO: Add more Menus
    enum class SimMenuItem { QUIT };  //// TODO: Add more functionalities
};

} // namespace OpenSimRT
