from __future__ import annotations

import shutil
import subprocess
from dataclasses import dataclass
from threading import Event, Thread


def audibly_notify(message: str) -> None:
    text = message.strip()
    if not text:
        return

    print("\a", end="", flush=True)

    speech_commands = (
        ["say", text],
        ["spd-say", text],
        ["espeak-ng", text],
        ["espeak", text],
    )
    for cmd in speech_commands:
        executable = cmd[0]
        if not shutil.which(executable):
            continue
        try:
            subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except OSError:
            continue
        break


@dataclass(slots=True)
class AudibleReminder:
    stop_event: Event
    thread: Thread

    def stop(self, *, timeout_s: float = 1.0) -> None:
        self.stop_event.set()
        if self.thread.is_alive():
            self.thread.join(timeout=timeout_s)


def start_periodic_audible_reminder(
    message: str,
    *,
    interval_s: float = 300.0,
) -> AudibleReminder:
    stop_event = Event()

    def _run() -> None:
        while not stop_event.wait(interval_s):
            if stop_event.is_set():
                break
            audibly_notify(message)

    thread = Thread(
        target=_run,
        name=f"audible-reminder:{message}",
        daemon=True,
    )
    thread.start()
    return AudibleReminder(stop_event=stop_event, thread=thread)
