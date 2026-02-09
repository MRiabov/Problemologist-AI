def rotate_to(current_angle: float, target_angle: float, kp: float = 1.0) -> float:
    """
    A simple proportional controller to rotate towards a target angle.
    Returns the control input (e.g. torque or velocity).
    """
    error = target_angle - current_angle
    # Normalize error to [-pi, pi] if needed, but for simple servos it's fine
    return kp * error
