from rdflib import URIRef

from bdd_exec_ros2.observation import PlanarContainmentEvaluator

_SCENE = "https://secorolab.github.io/models/demos/collab/scene/"
_DRAWER = URIRef(f"{_SCENE}drawer")
_WALL_WS = URIRef(f"{_SCENE}wall-ws")
_ROBOT_WS = URIRef(f"{_SCENE}robot-ws")
_TRAY_SIZE = (0.24, 0.20)
_MARGIN_M = 0.05
_ALLOWED_OUTSIDE_RATIO = 0.05

wall_ws_center_inside = PlanarContainmentEvaluator(
    _DRAWER, _WALL_WS, (1.40, 0.646), margin_m=_MARGIN_M
)
wall_ws_footprint_inside = PlanarContainmentEvaluator(
    _DRAWER,
    _WALL_WS,
    (1.40, 0.646),
    margin_m=_MARGIN_M,
    footprint_size_xy=_TRAY_SIZE,
    allowed_outside_ratio=_ALLOWED_OUTSIDE_RATIO,
)
robot_ws_center_inside = PlanarContainmentEvaluator(
    _DRAWER, _ROBOT_WS, (1.60, 0.80), margin_m=_MARGIN_M
)
robot_ws_footprint_inside = PlanarContainmentEvaluator(
    _DRAWER,
    _ROBOT_WS,
    (1.60, 0.80),
    margin_m=_MARGIN_M,
    footprint_size_xy=_TRAY_SIZE,
    allowed_outside_ratio=_ALLOWED_OUTSIDE_RATIO,
)
