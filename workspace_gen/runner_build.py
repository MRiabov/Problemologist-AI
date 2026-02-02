
import json
import sys
import os

sys.path.append("/workspace")
import template_build

seed = 42
scale = (1.0, 1.0, 1.0)

try:
    # Try calling with scale first (new signature)
    import inspect
    sig = inspect.signature(template_build.build)
    if "scale" in sig.parameters:
        res = template_build.build(seed, scale=scale)
    else:
        res = template_build.build(seed)
    print(f"BUILD_RESULT:{res}")
except Exception as e:
    print(f"BUILD_ERROR:{str(e)}")
