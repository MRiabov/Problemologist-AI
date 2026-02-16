import numpy as np

from shared.simulation.view_utils import get_best_isometric_view


def test_isometric_view_alignment():
    # Looking at top face
    normal = (0, 0, 1)
    view_matrix = get_best_isometric_view(normal)
    m = np.array(view_matrix)

    # Camera position should be one of the [1,1,1] family with Z > 0
    inv_m = np.linalg.inv(m)
    eye = inv_m[:3, 3]

    # eye / norm(eye) should have eye[2] > 0
    assert eye[2] > 0


def test_isometric_view_orthogonal():
    # Looking at side face
    normal = (1, 0, 0)
    view_matrix = get_best_isometric_view(normal)
    inv_m = np.linalg.inv(np.array(view_matrix))
    eye = inv_m[:3, 3]

    assert eye[0] > 0
