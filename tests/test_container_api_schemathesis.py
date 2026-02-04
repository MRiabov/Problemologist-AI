import sys
from pathlib import Path

from schemathesis.openapi import from_asgi

# Ensure src is in path so we can import the app
project_root = Path(__file__).resolve().parent.parent
if str(project_root) not in sys.path:
    sys.path.append(str(project_root))

from src.assets.scripts.container_agent import app

schema = from_asgi("/openapi.json", app)


@schema.parametrize()
def test_api(case):
    """
    Fuzz test the Container Agent API using Schemathesis.
    This validates that the application conforms to its OpenAPI schema
    and doesn't crash (500 errors) on arbitrary inputs.
    """
    case.call_and_validate()
