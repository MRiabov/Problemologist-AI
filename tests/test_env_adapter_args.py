import sys
import unittest
from unittest.mock import MagicMock

# Mock dependencies
sys.modules["src.environment.runtime"] = MagicMock()

mock_langchain = MagicMock()
sys.modules["langchain_core"] = mock_langchain
sys.modules["langchain_core.tools"] = mock_langchain

def tool_mock(func):
    import inspect
    sig = inspect.signature(func)
    class MockTool:
        pass
    m = MockTool()
    m.func = func
    m.params = sig.parameters
    class Schema:
        def schema(self):
            return {'properties': {k: {} for k in sig.parameters.keys()}}
    m.args_schema = Schema()
    return m

mock_langchain.tool = tool_mock

# Debug import
try:
    from src.agent.tools.env_adapter import verify_solution
except Exception as e:
    print(f"Import failed: {e}")
    # We might need to mock more things if verify_solution is not imported
    verify_solution = None

class TestEnvAdapter(unittest.TestCase):
    def test_verify_solution_schema(self):
        if verify_solution is None:
            self.fail("Could not import verify_solution")

        props = verify_solution.params

        required_fields = [
            'control_path',
            'design_file',
            'process',
            'target_quantity',
            'max_unit_cost'
        ]

        missing = [field for field in required_fields if field not in props]

        if missing:
            self.fail(f"Missing arguments in verify_solution schema: {missing}")

if __name__ == "__main__":
    unittest.main()
