import pytest

# Mark all tests in this directory as integration tests
# and force them into a single xdist group to prevent parallel simulation overload.
pytestmark = [pytest.mark.integration, pytest.mark.xdist_group(name="physics_sims")]
