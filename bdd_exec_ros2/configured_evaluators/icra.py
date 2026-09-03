from rdflib import URIRef

from bdd_exec_ros2.observation import (
    PlanarContainmentEvaluator,
    WrenchForceNormWithinLimitEvaluator,
    WrenchPeakForceNormWithinLimitEvaluator,
    WrenchRmsForceNormWithinLimitEvaluator,
)

_SCENE = "https://secorolab.github.io/models/demos/collab/scene/"
_DRAWER = URIRef(f"{_SCENE}drawer")
_WALL_WS = URIRef(f"{_SCENE}wall-ws")
_ROBOT_WS = URIRef(f"{_SCENE}robot-ws")
_TRAY_SIZE = (0.24, 0.20)
_MARGIN_M = 0.05
_ALLOWED_OUTSIDE_RATIO = 0.05
_SIM_COMPLIANCE_LIMIT_N = 1.3 * 10.0
_REAL_COMPLIANCE_LIMIT_N = 1.3 * 45.0

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

sim_force_instantaneous_within_limit = WrenchForceNormWithinLimitEvaluator(
    _SIM_COMPLIANCE_LIMIT_N
)
sim_force_peak_within_limit = WrenchPeakForceNormWithinLimitEvaluator(
    _SIM_COMPLIANCE_LIMIT_N
)
sim_force_rms_within_limit = WrenchRmsForceNormWithinLimitEvaluator(
    _SIM_COMPLIANCE_LIMIT_N
)
real_force_instantaneous_within_limit = WrenchForceNormWithinLimitEvaluator(
    _REAL_COMPLIANCE_LIMIT_N
)
real_force_peak_within_limit = WrenchPeakForceNormWithinLimitEvaluator(
    _REAL_COMPLIANCE_LIMIT_N
)
real_force_rms_within_limit = WrenchRmsForceNormWithinLimitEvaluator(
    _REAL_COMPLIANCE_LIMIT_N
)
