import genesis as gs
import threading
import time


def run_steps(scene, idx):
    print(f"Thread {idx}: Start Stepping")
    try:
        for i in range(10):
            scene.step()
            time.sleep(0.01)
        print(f"Thread {idx}: Done Stepping")
    except Exception as e:
        print(f"Thread {idx}: Step Failed: {e}")


if __name__ == "__main__":
    gs.init(backend=gs.cpu)

    print("Main: Create Scene 1")
    scene1 = gs.Scene(show_viewer=False)
    scene1.add_entity(gs.morphs.Plane())
    scene1.build()

    print("Main: Create Scene 2")
    scene2 = gs.Scene(show_viewer=False)
    scene2.add_entity(gs.morphs.Plane())
    scene2.build()

    t1 = threading.Thread(target=run_steps, args=(scene1, 1))
    t2 = threading.Thread(target=run_steps, args=(scene2, 2))

    t1.start()
    t2.start()

    t1.join()
    t2.join()
