import alembic
import build123d
import gymnasium
import mujoco
import numpy as np
import sqlalchemy


def test_imports():
    assert build123d.__version__ is not None
    assert mujoco.__version__ is not None
    assert gymnasium.__version__ is not None
    assert np.__version__ is not None
    assert sqlalchemy.__version__ is not None
    assert alembic.__version__ is not None


def test_hello():
    assert True
