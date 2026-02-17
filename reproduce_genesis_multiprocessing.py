import genesis as gs
import multiprocessing
import time
import os


def run_sim(idx):
    print(f"Process {idx} (PID {os.getpid()}): Init")
    try:
        gs.init(backend=gs.cpu)
    except Exception as e:
        print(f"Process {idx}: Init failed: {e}")

    print(f"Process {idx}: Create Scene")
    scene = gs.Scene(show_viewer=False)
    plane = scene.add_entity(gs.morphs.Plane())

    print(f"Process {idx}: Build Scene")
    try:
        scene.build()
        print(f"Process {idx}: Build Success")
    except Exception as e:
        print(f"Process {idx}: Build Failed: {e}")
        return

    for i in range(10):
        scene.step()
        time.sleep(0.01)

    print(f"Process {idx}: Done")


if __name__ == "__main__":
    # Genesis might need spawn start method to avoid fork issues with CUDA/OpenGL contexts (though cpu backend might be fine)
    try:
        multiprocessing.set_start_method("spawn")
    except RuntimeError:
        pass

    p1 = multiprocessing.Process(target=run_sim, args=(1,))
    p2 = multiprocessing.Process(target=run_sim, args=(2,))

    p1.start()
    p2.start()

    p1.join()
    p2.join()
