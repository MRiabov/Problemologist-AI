# CAD and other infrastructure; dependencies.

## CAD 

### Part metadata

Parts and Assemblies have metadata, e.g. `cots_id` for COTS parts, `material_id` for material parts, and others; e.g. `fixed: bool` for usage during benchmark generation.

Define a classes `PartMetadata` and `CompoundMetadata` that can store all properties related to them. Without these mandatory fields, validation will fail.

### Assigning a part to workbenches

The agent must assign a part manufacturing method to the part. If not, the parts can not be priced or sent for manufacturing. 

The verification is done by a method.

So suppose the agent's code is as follows:

```py
from build123d import *
from utils.models.schemas import PartMetadata
from utils.enums import ManufacturingMethod
from utils import validate_and_price 
# utils is a __init__.py gathering all utils importable by the agent, ideally. We don't want to force the agent to look through the codebase; in fact it can't because of read permissions. However, it should be doable through utils.

with BuildPart() as part_builder:
    Box(10,10,10)

part_builder.part.label="custom_label"
part_builder.part.metadata = PartMetadata(
    manufacturing_method=ManufacturingMethod.CNC,
    material_id="aluminum-6061"
)

validate_and_price(part_builder.part) # prints ("Part {label} is valid, the unit price at XYZ pcs is ...)
```

### Rendering

The project needs to render the models in images (for preview) and for rendering.

#### Rendering CAD

To simplify matters *(actually, I couldn't debug pyvista, so I'm doing this)*, the CAD will be rendered by porting to MuJoCo/Genesis and rendering in there. It'll also provide a unified view for the model.
Alternatively, we could use some simple GLB renderer, but MuJoCo/Gensis already do it for us!

Note that in genesis, some tricks can be achieved to not build the scene fully and only use it for one-off rendering.

#### Rendering views

I presume the model will need to render a view or a set of views to get an understanding of what's happening during the simulation. Allow an extra `view_angles` parameter on `simulate` to trigger simulation from different sides, which would essentially reposition a camera (or a multiple) to render.

### Workbench technical details

Technical details of manufacturability constraints are discussed in spec 004 (not to be discussed here; however manufacturability is determined by deterministic algorithms.)

The workbench validation (as well as other util infrastructure are read-only in the container)

### Supported workbenches

3D printing, CNC and injection molding are supported.

<!-- In the future, it's very interesting to support topology optimization, but that's a separate project. -->

### Off-the-shelf parts (COTS)

It is self-understanding that engineers will use off-the-shelf parts - motors, fasteners, gears, etc. The catalog is well-defined in spec 007, but the model should have an access to a CLI tool or a set of python scripts to find something in a codebase. Again, we use DSPy.ReAct.

I suggest using a subagent for this. Give a prompt of what's necessary and let the subagent execute a series of read-only SQL prompts over a catalog DB. The agent will return a series of catalogs to use.

Both planner agent and engineer can only prompt the searching agent for searching.


## Other infra

### Strict schema

We will run `schemathesis` checks against the OpenAPI. Strictly type all schema to avoid ANY issues.

We use Pydantic and Beartype for this.

#### Beartype

To further avoid any issues, we will use Beartype for type checking.

### Schema autogeneration

We autogenerate python schemas, keeping in sync to the workers. We keep schemas defined in the Controller app; worker-light, worker-heavy, and frontend inherit them (for now). We have git hooks that implement the model.
Trace schema contract includes optional reasoning metadata fields (`reasoning_step_index`, `reasoning_source`) and generated clients must preserve those fields.

### Logging 
We choose Structlog because it looks better and easier to trace; also theoretically works with OpenTelemetry out of the box.
For utils used internally in an agent, a simple `logging` is acceptable too.



<!--
## CAD and and design validation

As said, "agents will live inside of a filesystem". The agents will generate and execute design validations of files in the filesystem.-->
