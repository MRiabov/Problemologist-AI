import numpy as np

# Type alias for clarity
Vector3 = tuple[float, float, float]
Matrix4x4 = list[list[float]]


def get_best_isometric_view(normal: Vector3) -> Matrix4x4:
    """
    Finds the best isometric view angle for a given normal vector.
    Returns a camera matrix (view matrix).
    """
    # 8 standard isometric directions
    isometric_vectors = []
    for x in [-1, 1]:
        for y in [-1, 1]:
            for z in [-1, 1]:
                v = np.array([x, y, z], dtype=float)
                isometric_vectors.append(v / np.linalg.norm(v))

    target_normal = np.array(normal, dtype=float)
    if np.linalg.norm(target_normal) > 0:
        target_normal /= np.linalg.norm(target_normal)

    # Find vector with highest dot product (most aligned)
    best_v = isometric_vectors[0]
    best_dot = -2.0

    for v in isometric_vectors:
        dot = np.dot(target_normal, v)
        if dot > best_dot:
            best_dot = dot
            best_v = v

    # Now generate a view matrix looking from best_v towards origin
    # We'll use a standard look-at matrix logic
    # Camera position: some arbitrary distance along best_v
    distance = 5.0
    eye = best_v * distance
    center = np.array([0, 0, 0], dtype=float)

    # Up vector: usually Z is up, but if we are looking along Z, use Y
    up = np.array([0, 0, 1], dtype=float)
    if abs(np.dot(best_v, up)) > 0.99:
        up = np.array([0, 1, 0], dtype=float)

    # LookAt matrix calculation
    f = center - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    s /= np.linalg.norm(s)
    u = np.cross(s, f)

    m = np.eye(4)
    m[0, 0:3] = s
    m[1, 0:3] = u
    m[2, 0:3] = -f
    m[0, 3] = -np.dot(s, eye)
    m[1, 3] = -np.dot(u, eye)
    m[2, 3] = np.dot(f, eye)

    return m.tolist()
