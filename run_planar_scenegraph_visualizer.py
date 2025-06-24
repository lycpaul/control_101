"""
Run examples of MeshcatVisualizer, e.g. to visualize a pendulum.
Usage demo: load a URDF, rig it up with a constant torque input, and draw it.
"""

import argparse

import numpy as np

from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    Simulator,
    StartMeshcat,
)
from pydrake.common import temp_directory
from pydrake.examples import ManipulationStation
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph


def run_pendulum_example(args):
    # Start meshcat server
    meshcat = StartMeshcat()
    
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    parser = Parser(plant)
    parser.AddModelsFromUrl(
        url="package://drake/examples/pendulum/Pendulum.urdf")
    plant.Finalize()

    # Add Meshcat visualizer
    meshcat_viz = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    
    if args.playback:
        meshcat_viz.StartRecording()

    diagram = builder.Build()
    simulator = Simulator(diagram) 
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)

    # Fix the input port to zero.
    plant_context = diagram.GetMutableSubsystemContext(
        plant, simulator.get_mutable_context())
    plant.get_actuation_input_port().FixValue(
        plant_context, np.zeros(plant.num_actuators()))
    plant_context.SetContinuousState(np.array([0.5, 0.1]))
    simulator.AdvanceTo(args.duration)

    if args.playback:
        meshcat_viz.StopRecording()
        meshcat_viz.PublishRecording()


def run_manipulation_example(args):
    # Start meshcat server
    meshcat = StartMeshcat()
    
    builder = DiagramBuilder()
    station = builder.AddSystem(ManipulationStation(time_step=0.005))
    station.SetupClutterClearingStation()
    station.Finalize()

    scene_graph = station.get_scene_graph()

    # Add Meshcat visualizer
    meshcat_viz = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)

    if args.playback:
        meshcat_viz.StartRecording()

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)

    # Fix the control inputs to zero.
    station_context = diagram.GetMutableSubsystemContext(
        station, simulator.get_mutable_context())
    station.GetInputPort("iiwa_position").FixValue(
        station_context, station.GetIiwaPosition(station_context))
    station.GetInputPort("iiwa_feedforward_torque").FixValue(
        station_context, np.zeros(7))
    station.GetInputPort("wsg_position").FixValue(
        station_context, np.zeros(1))
    station.GetInputPort("wsg_force_limit").FixValue(
        station_context, np.array([40.0]))
    simulator.AdvanceTo(args.duration)

    if args.playback:
        meshcat_viz.StopRecording()
        meshcat_viz.PublishRecording()


def main():
    np.set_printoptions(precision=5, suppress=True)
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("-T", "--duration",
                        type=float,
                        help="Duration to run sim in seconds.",
                        default=1.0)
    parser.add_argument("-m", "--models",
                        type=str,
                        nargs="*",
                        help="Models to run, at least one of [pend, manip]",
                        default=["pend"])
    parser.add_argument("-p", "--playback",
                        action="store_true",
                        help="Whether to record and playback the animation")
    args = parser.parse_args()

    for model in args.models:
        if model == "pend":
            run_pendulum_example(args)
        elif model == "manip":
            run_manipulation_example(args)
        else:
            print("Unrecognized model %s." % model)
            parser.print_usage()
            exit(1)


if __name__ == "__main__":
    main()
