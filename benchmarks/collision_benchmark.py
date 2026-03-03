import time
import random
from build123d import Box, Compound, Pos
from worker_heavy.utils.validation import validate

def create_non_overlapping_boxes(n):
    boxes = []
    for i in range(n):
        # Place boxes in a grid to avoid overlap
        x = (i % 10) * 20
        y = ((i // 10) % 10) * 20
        z = (i // 100) * 20
        boxes.append(Pos(x, y, z) * Box(10, 10, 10))
    return Compound(children=boxes)

def benchmark_validate(n_solids, overlap=False):
    print(f"Benchmarking with {n_solids} solids (Overlap: {overlap})...")
    if overlap:
        boxes = [Box(10, 10, 10) for _ in range(n_solids)]
        comp = Compound(children=boxes)
    else:
        comp = create_non_overlapping_boxes(n_solids)

    start_time = time.perf_counter()
    success, msg = validate(comp)
    end_time = time.perf_counter()

    duration = end_time - start_time
    print(f"Validation took {duration:.4f} seconds (Success: {success}, Msg: {msg})")
    return duration

if __name__ == "__main__":
    results = []
    for n in [10, 20, 50, 100]:
        duration = benchmark_validate(n)
        results.append((n, duration))

    # Test overlap detection correctness
    benchmark_validate(2, overlap=True)

    print("\nSummary:")
    print("N solids | Time (s)")
    print("---------|---------")
    for n, duration in results:
        print(f"{n:8d} | {duration:8.4f}")
