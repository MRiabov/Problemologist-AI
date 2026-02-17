import genesis as gs
import threading
import time


# Global lock to mimic GenesisBackend.sim_lock
GLOBAL_LOCK = threading.Lock()


def run_sim(idx):
    print(f"Thread {idx}: Init")
    # gs.init is already robust or called once in main, but let's be safe

    print(f"Thread {idx}: Create Scene (Acquiring Lock)")
    with GLOBAL_LOCK:
        print(f"Thread {idx}: Lock Acquired")
        try:
            # Re-init check inside lock if needed, though gs.init is global
            gs.init(backend=gs.cpu)
        except Exception:
            pass

        scene = gs.Scene(show_viewer=False)
        plane = scene.add_entity(gs.morphs.Plane())

        print(f"Thread {idx}: Build Scene")
        try:
            scene.build()
            print(f"Thread {idx}: Build Success")
        except Exception as e:
            print(f"Thread {idx}: Build Failed: {e}")
            return
    print(f"Thread {idx}: Lock Released")

    for i in range(10):
        scene.step()
        time.sleep(0.01)

    print(f"Thread {idx}: Done")


if __name__ == "__main__":
    gs.init(backend=gs.cpu)

    t1 = threading.Thread(target=run_sim, args=(1,))
    t2 = threading.Thread(target=run_sim, args=(2,))

    t1.start()
    t2.start()

    t1.join()
    t2.join()
