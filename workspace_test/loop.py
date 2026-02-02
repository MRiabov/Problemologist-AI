import os
import sys
import threading
import time
from contextlib import suppress


def self_destruct(delay: int):
    """Kills the process after delay seconds."""
    time.sleep(delay)
    # Using os._exit to bypass any try/except or finally blocks in the main thread
    os._exit(1)


if __name__ == "__main__":
    # Safety timeout of 60 seconds
    timeout = 60
    if len(sys.argv) > 1:
        with suppress(ValueError):
            timeout = int(sys.argv[1])

    threading.Thread(target=self_destruct, args=(timeout,), daemon=True).start()

    # Infinite loop to simulate hanging code
    while True:
        pass
