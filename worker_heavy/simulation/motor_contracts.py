from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping

from shared.cots.parts.motors import retrieve_cots_physics
from shared.enums import MotorControlMode

SUPPORTED_MOTOR_CONTROL_MODES = {mode.value for mode in MotorControlMode}
CONTROL_MODE_TO_ACTUATOR_TYPE = {
    MotorControlMode.CONSTANT.value: "velocity",
    MotorControlMode.SINUSOIDAL.value: "position",
    MotorControlMode.ON_OFF.value: "motor",
}
DOF_TO_JOINT_CONTRACT = {
    "rotate_x": ("hinge", [1.0, 0.0, 0.0]),
    "rotate_y": ("hinge", [0.0, 1.0, 0.0]),
    "rotate_z": ("hinge", [0.0, 0.0, 1.0]),
    "slide_x": ("slide", [1.0, 0.0, 0.0]),
    "slide_y": ("slide", [0.0, 1.0, 0.0]),
    "slide_z": ("slide", [0.0, 0.0, 1.0]),
}


def _control_value(control: Any, key: str, default: Any = None) -> Any:
    if control is None:
        return default
    if isinstance(control, Mapping):
        return control.get(key, default)
    return getattr(control, key, default)


def _normalize_text(value: Any) -> str:
    text = str(value or "").strip()
    if not text:
        raise ValueError("value must be a non-empty string")
    return text


def resolve_solution_motor_joint_contract(
    *, part_name: str, dofs: list[str] | tuple[str, ...]
) -> tuple[str, list[float]]:
    normalized_part_name = _normalize_text(part_name)
    normalized_dofs = [
        _normalize_text(dof).lower()
        for dof in dofs
    ]
    if len(normalized_dofs) != 1:
        raise ValueError(
            f"motor '{normalized_part_name}' must declare exactly one DOF token"
        )

    dof_token = normalized_dofs[0]
    joint_contract = DOF_TO_JOINT_CONTRACT.get(dof_token)
    if joint_contract is None:
        raise ValueError(
            f"motor '{normalized_part_name}' uses unsupported DOF token '{dof_token}'"
        )
    joint_type, joint_axis = joint_contract
    return joint_type, list(joint_axis)


@dataclass(frozen=True)
class ResolvedMotorContract:
    part_name: str
    cots_id: str
    joint_name: str
    actuator_type: str
    force_range: tuple[float, float]
    max_velocity: float
    control_mode: str
    control_speed: float
    control_frequency: float | None

    def to_scene_record(self, *, dofs: list[str], moving_part_type: str) -> dict[str, Any]:
        record: dict[str, Any] = {
            "part_name": self.part_name,
            "name": self.part_name,
            "actuator_name": self.part_name,
            "type": moving_part_type,
            "dofs": dofs,
            "joint": self.joint_name,
            "cots_id": self.cots_id,
            "control": {
                "mode": self.control_mode,
                "speed": self.control_speed,
            },
            "actuator_type": self.actuator_type,
            "force_range": [self.force_range[0], self.force_range[1]],
            "max_velocity": self.max_velocity,
        }
        if self.control_frequency is not None:
            record["control"]["frequency"] = self.control_frequency
        return record


def resolve_solution_motor_contract(
    *,
    part_name: str,
    cots_id: str | None,
    control: Any | None,
    joint_name: str | None = None,
) -> ResolvedMotorContract:
    """Resolve a motorized moving-part contract into backend-facing values."""

    normalized_part_name = _normalize_text(part_name)
    normalized_cots_id = _normalize_text(cots_id)

    control_mode_raw = _control_value(control, "mode")
    control_mode = _normalize_text(control_mode_raw).upper()
    if control_mode not in SUPPORTED_MOTOR_CONTROL_MODES:
        raise ValueError(
            f"unsupported motor control mode '{control_mode_raw}' for "
            f"moving part '{normalized_part_name}'"
        )

    try:
        control_speed = float(_control_value(control, "speed"))
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"motor '{normalized_part_name}' must declare a numeric control speed"
        ) from exc
    if not math.isfinite(control_speed) or control_speed <= 0:
        raise ValueError(
            f"motor '{normalized_part_name}' must use a positive control speed"
        )

    frequency_raw = _control_value(control, "frequency")
    control_frequency: float | None = None
    if frequency_raw is not None:
        try:
            control_frequency = float(frequency_raw)
        except (TypeError, ValueError) as exc:
            raise ValueError(
                f"motor '{normalized_part_name}' has an invalid control frequency"
            ) from exc
        if not math.isfinite(control_frequency) or control_frequency <= 0:
            raise ValueError(
                f"motor '{normalized_part_name}' must use a positive control frequency"
            )

    physics = retrieve_cots_physics(normalized_cots_id)
    if physics is None:
        raise ValueError(
            f"motor '{normalized_part_name}' must map to a supported COTS motor"
        )

    torque = float(physics["torque"])
    max_velocity = float(physics["max_velocity"])
    actuator_type = CONTROL_MODE_TO_ACTUATOR_TYPE[control_mode]
    normalized_joint_name = _normalize_text(joint_name or f"{normalized_part_name}_joint")

    return ResolvedMotorContract(
        part_name=normalized_part_name,
        cots_id=normalized_cots_id,
        joint_name=normalized_joint_name,
        actuator_type=actuator_type,
        force_range=(-torque, torque),
        max_velocity=max_velocity,
        control_mode=control_mode,
        control_speed=control_speed,
        control_frequency=control_frequency,
    )
