import threading
import os
import time
import sys


def self_destruct(delay):
    """Kills the process after delay seconds."""
    time.sleep(delay)
    # Using os._exit to bypass any try/except or finally blocks in the main thread
    os._exit(1)


if __name__ == "__main__":
    # Safety timeout of 600 seconds
    timeout = 600
    if len(sys.argv) > 1:
        try:
            timeout = int(sys.argv[1])
        except ValueError:
            pass

    threading.Thread(target=self_destruct, args=(timeout,), daemon=True).start()

    # Infinite loop to simulate hanging code
    while True:
        pass
