"""Inspect a simulator or load a SceneInstance through simulation_interfaces."""

import argparse
from enum import StrEnum
from pathlib import Path
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rdflib import RDF, URIRef
from scene_dsl.langs import scenex_metamodel
from rdf_utils.models.vocab import URI_EXEC_TYPE_SCENE_INST
from scene_dsl.rdf.scenex import create_scenex_model_graph
from scene_dsl.rdf_parser.scenex import SceneInstanceModel

from bdd_exec_ros2.sim_interfaces import (
    FEATURE_NAMES,
    SimInterface,
)


class Command(StrEnum):
    LIST_FEATURES = "list-features"
    LOAD_SCENE = "load-scene"


def _parse_args(args=None):
    parser = argparse.ArgumentParser(
        description="Inspect or load a simulation scene",
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
    options = parser.parse_args(args)
    if options.command == Command.LOAD_SCENE and options.scene_model is None:
        parser.error("load-scene requires --scene-model")
    if options.command == Command.LIST_FEATURES and options.scene_model is not None:
        parser.error("--scene-model is only valid with load-scene")
    return options


def main(args=None):
    raw_args = sys.argv if args is None else ["sim_interface_test", *args]
    options = _parse_args(remove_ros_args(raw_args)[1:])

    rclpy.init(args=raw_args)
    node = Node("sim_interface_test", namespace=options.service_namespace)
    sim_intf = SimInterface(node=node)
    try:
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

        if options.command == Command.LOAD_SCENE:
            model = scenex_metamodel().model_from_file(options.scene_model)
            graph = create_scenex_model_graph(model)
            scene_ids = list(
                graph.subjects(RDF.type, URI_EXEC_TYPE_SCENE_INST, unique=True)
            )
            if len(scene_ids) != 1:
                raise ValueError(f"expected one scene instance URI, found {scene_ids}")
            scn_inst_id = scene_ids[0]
            assert isinstance(scn_inst_id, URIRef)

            scene = SceneInstanceModel(scn_inst_id=scn_inst_id, graph=graph)
            task = rclpy.get_global_executor().create_task(sim_intf.load_world(scene))
            rclpy.spin_until_future_complete(node, task)
            path = task.result()
            node.get_logger().info(f"Loaded world {path}")
            return

        raise ValueError(f"unhandled command: {options.command}")

    finally:
        node.destroy_node()
        rclpy.shutdown()
