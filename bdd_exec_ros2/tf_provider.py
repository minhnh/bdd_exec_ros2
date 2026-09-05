from __future__ import annotations

from collections.abc import Callable, Iterable

from bdd_dsl.models.observation import read_string_entity_mappings
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.time import Time
from rdf_utils.models.vocab import (
    URI_OBS_PRED_ENTITY_MAPPER,
    URI_OBS_PRED_PROVIDER,
    URI_ROS_PRED_REFERENCE_FRAME,
)
from rdflib import Graph, URIRef
from tf2_ros import Buffer, TransformException, TransformListener


class TfPollingHandle:
    def __init__(
        self,
        provider: TfProvider,
        reference_frame: str,
        frame_names: Iterable[str],
        frequency: float,
        callback: Callable[[dict[str, PoseStamped]], None],
    ) -> None:
        if frequency <= 0:
            raise ValueError("TF polling frequency must be positive")
        self._provider = provider
        self._reference_frame = reference_frame
        self._frame_names = tuple(frame_names)
        self._callback = callback
        self._cancelled = False
        self._timer = provider._node.create_timer(
            1.0 / frequency, self._poll, callback_group=provider._callback_group
        )
        self._poll()

    def _poll(self) -> None:
        if self._cancelled:
            return
        poses = self._provider.get_poses(self._reference_frame, self._frame_names)
        if poses:
            self._callback(poses)

    def cancel(self) -> None:
        if not self._cancelled:
            self._cancelled = True
            self._provider._node.destroy_timer(self._timer)


class TfProvider:
    """Read the TF frames requested by observation entity mappers."""

    def __init__(self, node: Node, callback_group: CallbackGroup | None = None) -> None:
        self._node = node
        self._logger = node.get_logger()
        self._callback_group = callback_group
        self._buffer = Buffer(node=node)
        self._listener = TransformListener(self._buffer, node)

    def get_poses(
        self, reference_frame: str, frame_names: Iterable[str]
    ) -> dict[str, PoseStamped]:
        poses = {}
        for frame_name in frame_names:
            try:
                transform = self._buffer.lookup_transform(
                    reference_frame, frame_name, Time()
                )
            except TransformException as exc:
                self._logger.warning(
                    f"could not resolve TF '{reference_frame}' to '{frame_name}': {exc}",
                    throttle_duration_sec=1.0,
                )
                continue
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            poses[frame_name] = pose
        return poses

    def start_pose_polling(
        self,
        graph: Graph,
        provider_uri: URIRef,
        frequency: float,
        callback: Callable[[dict[str, PoseStamped]], None],
    ) -> TfPollingHandle:
        reference_frame = graph.value(
            provider_uri, URI_ROS_PRED_REFERENCE_FRAME, any=False
        )
        if reference_frame is None:
            raise ValueError(f"TF provider '{provider_uri}' has no reference frame")
        frame_names = sorted(
            {
                frame_name
                for observation_uri in graph.subjects(
                    URI_OBS_PRED_PROVIDER, provider_uri
                )
                for mapper_uri in graph.objects(
                    observation_uri, URI_OBS_PRED_ENTITY_MAPPER
                )
                for frame_name, _ in read_string_entity_mappings(graph, mapper_uri)
            }
        )
        if not frame_names:
            raise ValueError(f"TF provider '{provider_uri}' has no mapped frames")
        return TfPollingHandle(
            self, str(reference_frame), frame_names, frequency, callback
        )
