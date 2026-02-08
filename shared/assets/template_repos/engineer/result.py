from build123d import *
from worker.utils.dfm import validate_and_price
from worker.utils.validation import simulate
from worker.utils.handover import submit_for_review
from worker.utils.docs import get_docs_for
from worker.workbenches.models import ManufacturingMethod, ManufacturingConfig

# Define your solution part/assembly
# ... your code here ...

# Create the final compound
# solution = Compound(children=[...])

# Configure manufacturing
# config = ManufacturingConfig(...)

# Validate and Price
# result = validate_and_price(solution, ManufacturingMethod.CNC, config)

# if result.is_manufacturable:
#     print(f"Cost: {result.cost}")
#     # Simulate if valid
#     sim_result = simulate(solution)
#     if sim_result.success:
#         submit_for_review(solution)
#     else:
#         print(f"Simulation failed: {sim_result.summary}")
# else:
#     print(f"DFM failed: {result.violations}")
