# Copyright 2026 Minh Nguyen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Inspect, reset, or load a simulator through simulation_interfaces."""

import argparse
import sys
from enum import StrEnum
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rdf_utils.uri import try_expand_curie
from scene_dsl.langs import scenex_metamodel
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.scenex import SceneInstanceModel

from bdd_exec_ros2.sim_interfaces import (
    FEATURE_NAMES,
    SimInterface,
)


class Command(StrEnum):
    GET_POSE = "get-pose"
    LIST_FEATURES = "list-features"
    LOAD_SCENE = "load-scene"
    RESET = "reset"


def _parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Inspect, reset, or load a simulation scene",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("command", type=Command, choices=Command)
    parser.add_argument(
        "--service-namespace",
        default="/",
        help="namespace containing the simulation interface services",
    )
    parser.add_argument(
        "--scene-model",
        type=Path,
        default=None,
        help="executable Scene DSL model",
    )
    parser.add_argument(
        "--world-entity-name",
        default="world",
        help="scene resource entity mapped to the simulator world frame",
    )
    parser.add_argument(
        "--element-id",
        default=None,
        help="CURIE of the scene element whose pose should be queried",
    )
    options = parser.parse_args(args)
    model_commands = {Command.GET_POSE, Command.LOAD_SCENE}
    if options.command in model_commands and options.scene_model is None:
        parser.error(f"{options.command} requires --scene-model")
    if options.command not in model_commands and options.scene_model is not None:
        parser.error("--scene-model is only valid with get-pose or load-scene")
    if options.command == Command.GET_POSE and options.element_id is None:
        parser.error("get-pose requires --element-id")
    if options.command != Command.GET_POSE and options.element_id is not None:
        parser.error("--element-id is only valid with get-pose")
    return options


def main(args=None):
    raw_args = sys.argv if args is None else ["sim_interface_test", *args]
    options = _parse_args(remove_ros_args(raw_args)[1:])

    rclpy.init(args=raw_args)
    node = Node("sim_interface_test", namespace=options.service_namespace)
    try:
        graph = None
        if options.command in {Command.GET_POSE, Command.LOAD_SCENE}:
            model = scenex_metamodel().model_from_file(options.scene_model)
            graph = create_scenex_model_graph(model)

        sim_intf = SimInterface(
            node=node,
            model_graph=graph,
            world_entity_name=options.world_entity_name,
        )

        if options.command == Command.LIST_FEATURES:
            task = rclpy.get_global_executor().create_task(
                sim_intf.get_sim_features(quiet=False)
            )
            rclpy.spin_until_future_complete(node, task)
            features = task.result()
            if features is None:
                return
            print("Spawn formats:", ", ".join(features.spawn_formats) or "(none)")
            print("Custom info:", features.custom_info or "(none)")
            print("Features:")
            for feat_num in features.features:
                feat_name = FEATURE_NAMES.get(feat_num, f"UNKNOWN({feat_num})")
                print(f"- {feat_name}")
            return

        if options.command == Command.RESET:
            task = rclpy.get_global_executor().create_task(sim_intf.reset_simulation())
            rclpy.spin_until_future_complete(node, task)
            task.result()
            node.get_logger().info("Reset simulation")
            return

        if options.command in {Command.GET_POSE, Command.LOAD_SCENE}:
            if not model.scene_insts:
                raise ValueError(f"'{options.scene_model}' has no scene instances")
            scene = SceneInstanceModel(model.scene_insts[0].uri, graph)

        if options.command == Command.GET_POSE:
            element_id = try_expand_curie(
                graph.namespace_manager,
                options.element_id,
            )
            assert element_id is not None
            task = rclpy.get_global_executor().create_task(
                sim_intf.get_element_pose(scene, element_id)
            )
            rclpy.spin_until_future_complete(node, task)
            pose = task.result()
            if pose is None:
                node.get_logger().warning(f"No pose found for '{element_id}'")
            else:
                print(pose)
            return

        if options.command == Command.LOAD_SCENE:
            task = rclpy.get_global_executor().create_task(sim_intf.setup_scene(scene))
            rclpy.spin_until_future_complete(node, task)
            node.get_logger().info(f"Spawned entities {task.result()}")
            return

        raise ValueError(f"unhandled command: {options.command}")

    finally:
        node.destroy_node()
        rclpy.shutdown()
