"""config/gate_params.yaml 의 게이트 LOS 추력이 gate_follower 코드 기본값과 같은지 검사한다."""

from pathlib import Path

import yaml

from kaboat_behaviors.gate_follower import GATE_LOS_THRUST_RATIO
from kaboat_behaviors.los_controller import LOS_TURN_RATIO

CONFIG = (Path(__file__).resolve().parents[2]
          / 'kaboat_bringup' / 'config' / 'gate_params.yaml')


def test_gate_los_forward_is_point_three_five_in_code_and_yaml():
    params = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'gate_follower']['ros__parameters']
    assert GATE_LOS_THRUST_RATIO == 0.35
    assert params['gate.los_thrust_ratio'] == GATE_LOS_THRUST_RATIO
    assert params['gate.los_turn_ratio'] == LOS_TURN_RATIO   # 회전 상한은 공용 0.31 그대로


def test_gate_corridor_values_in_yaml_match_code_defaults():
    """현장 튜닝은 코드 기본값과 yaml 을 같이 바꾼다 — 게이트 사슬·부표 기억 포함."""
    import dataclasses

    from kaboat_behaviors.gate_corridor import GateParams

    params = yaml.safe_load(CONFIG.read_text(encoding='utf-8'))[
        'gate_follower']['ros__parameters']
    defaults = GateParams()
    for field in dataclasses.fields(GateParams):
        key = f'gate.{field.name}'
        if key in params:
            assert params[key] == getattr(defaults, field.name), key
    for name in ('max_chain_gates', 'check_horizon', 'memory_match_radius',
                 'memory_confirm_hits', 'memory_forget_sec', 'memory_recheck_range',
                 'memory_prune_range'):
        assert f'gate.{name}' in params, name
