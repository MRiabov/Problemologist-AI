from build123d import Compound, Location, Sphere

from shared.enums import ManufacturingMethod
from utils.metadata import CompoundMetadata, PartMetadata
from utils.submission import simulate, submit_for_review, validate


def build() -> Compound:
    guide = Sphere(1.0)
    guide = guide.move(Location((8.0, 0.0, 1.0)))
    guide.label = "projectile_ball"
    guide.metadata = PartMetadata(
        material_id="abs",
        fixed=False,
        manufacturing_method=ManufacturingMethod.THREE_DP,
    )

    scene = Compound(children=[guide])
    scene.label = "benchmark_scene"
    scene.metadata = CompoundMetadata(fixed=False)
    return scene


result = build()

if __name__ == "__main__":
    validate_ok, validate_message = validate(result)
    print(validate_ok)
    print(validate_message)

    sim_result = simulate(result)
    print(sim_result.success)
    print(sim_result.message)

    if validate_ok and sim_result.success:
        print(submit_for_review(result))
