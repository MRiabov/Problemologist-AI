import httpx
import uuid
import time

WORKER_URL = "http://localhost:8001"
SESSION_ID = f"test_sess_{uuid.uuid4().hex[:8]}"

SCRIPT_CONTENT = """
from build123d import *
def build():
    with BuildPart() as p:
        Box(10, 10, 10)
    return p.part
"""


def reproduce():
    with httpx.Client(base_url=WORKER_URL, timeout=60.0) as client:
        print(f"Testing session: {SESSION_ID}")

        # 1. Simulate
        payload = {"script_content": SCRIPT_CONTENT}
        headers = {"X-Session-Id": SESSION_ID}

        print("Calling /benchmark/simulate...")
        resp = client.post("/benchmark/simulate", json=payload, headers=headers)

        if resp.status_code != 200:
            print(f"Error: {resp.status_code}")
            print(resp.text)
            return

        data = resp.json()
        print(f"Success: {data['success']}")
        print(f"Message: {data['message']}")

        if data["artifacts"]:
            print("Artifacts generated:")
            for k, v in data["artifacts"].items():
                if isinstance(v, list):
                    print(f"  {k}: {len(v)} items")
                else:
                    print(f"  {k}: {'exists' if v else 'empty'}")


if __name__ == "__main__":
    reproduce()
