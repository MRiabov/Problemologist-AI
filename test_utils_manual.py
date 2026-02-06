from utils import simulate, validate_and_price
from build123d import Box
import os

def test():
    print("Testing validate_and_price...")
    box = Box(10, 10, 10)
    res = validate_and_price(box)
    print(f"Validation Result: {res}")

    print("
Testing simulate...")
    # Mocking environment for simulate
    os.environ["CONTROLLER_URL"] = "http://localhost:8000" # Dummy
    res_sim = simulate(box)
    print(f"Simulation Result: {res_sim}")

if __name__ == "__main__":
    test()
