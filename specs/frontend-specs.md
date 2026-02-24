# Frontend specs

As frontend has grown in the complexity and becomes an actual user-facing feature, it is worth separating it into the UI.
The main architectural specification is in @./desired_architecture.md

## Core flows

The frontend has two core flows:

1. Benchmark generation
2. Problem solution (a.k.a. Engineering.)

In both, engineer submit requests one-by-one as a chat UI interface common in coding agents.

### Benchmark generation workflow

For engineers, benchmarks serve as a way to create a set of constraints that the model must follow - "problem" geometric constraints and unit cost/weight/manufacturing solutions constraints.

To specify a benchmark, the engineers will define how the benchmark will behave, or at least how it looks (see main `desired_architecture.md` for details), and the model will implement it.

#### Confirming the workflow

Exact logic as in the "Solution workflow".

### Solution workflow

Engineers must be able to prompt solutions to benchmarks (this is a core workflow). They will select a benchmark that will be solved and will generate solutions to the benchmark.
The planning will then start, the engineers will be able to review edits, and the models will be generated from there.

### Shared workflow parts

The UI for benchmark generation and for solution is likely exactly equal, consisting of:

1. A history of sessions,
2. A coding agent chat interface,
3. A 3d CAD viewer,
4. A code file viewer.

Thus, they share most of equal features.

### Plan approval

Plans are approved or commented on explicitly by the user. After the planning is done in the solution/benchmark workflow, the users are to be able to review and confirm/disapprove the plan, and share comments in the chat section. Confirmation is done by a button in the bottom of a chat interface, or in the top right top of the file explorer.

### Chat UI

In Chat UI, agents must be able to:

1. View agents' reasoning traces
2. View agents' tool calls (read, write)
3. Interrupt agents before they finish.
4. Steer agents reasoning in a "chat" mode, meaning, correct their reasoning in case they made an incorrect assumption.

#### Reasoning traces

By default, agents' reasoning is hidden, however users are able to expand it. [Chat UI desired design](/frontend/designs/Chat%20UI%20design.png)

### Interrupting the worker

The engineers will have a "stop" button, superseding the "send" message button, but only active during agent generation.
*Note:* if we want to stop the generation in the controller, it will also halt the job(s) in the workers.

## Component layout

Benchmark generator and engineer viewer have a very similar structure:

We separate the entire UI to 3 columns about 3:3:6 split - 3 is a sidebar column with previous session history, 3 for the Chat UI, and 6 for the rightmost column - the CAD viewer and filesystem, split verticaly.
      - Note! This is by default. We allow to resize the dashboard pieces.

It is likely we will use the exact same component/template for the benchmark and engineer views.

## CAD and simulation viewer

A CAD viewer is a tool that engineers will use to:

1. View the model
2. View topology (on by default, hideable)
3. View simulations (time-progressive)
     - Move the simulation time forward or backward.
4. View individual parts (hide some or all other parts):
     - During design
     - During simulation (make some parts hideable)
5. Suggest improvements on the model.
    - Add parts to context. Click the part or a face to reference it in during the next workflow

Ideally, this the CAD viewer renders build123d directly as WASM in the browser, however, I don't want to debug it yet, so the GLB/OBJ model(s) will be pulled from the backend. (or STEP models? I'm not sure. Whatever YACV supports.)

We will use standard GLB files exported from the backend. To support topology selection (faces, edges), the backend will break down the model into individual meshes for each face/edge within the GLB file, named accordingly (e.g., `face_1`, `edge_2`).

Assets (GLB, OBJ, images, etc.) are served by the controller, which acts as a proxy to the worker filesystem or pulls from S3 storage. The frontend maintains a "controller-first" communication model for all requests.

Viewing topology, the model, is done as in the standard CAD viewers. Viewing the simulation modifies the screen (to possibly a non-CAD viewer? to show simulation details)

### Viewing the simulation

Notably, simlations are sometimes rigid-body and sometimes (most of the time) deformable meshes. This means that the simulator viewer must support deformable meshes.

In addition, the simulation is time-progressive, and that the users are to be able to rewind/skip kind of like they can in video viewers.

We will use mostly `three.js` with wrappers to display simulation.

### Selecting the CAD models

The users are to be able to click on a CAD model and do actions with it, primarily show/hide. There will be a way to select if the user wants to:

1. Select a part
2. Particular primitive (face, vertex, line/arc.)
3. Or a subassembly.

Thus three buttons.

### Viewing electronics and circuits

We have circuit design in our application. The engineers are to be able to view circuits in the application.

We use `tscircuit` as our dependency. (actively maintained, ships with a lot of features out of the box, including, circuit rendering and even, PCB rendering).

Why not SVG rendering: the users should be able to click and add the particular part to a context.

## Design

A super-modern design to a degree, suitable for an engineering software, however signficantly more "modern" than incumbents.

### Color palette

Light mode: White with black (or just dark) as a primary color,
Dark mode: Dark with white as a primary color

The users are to be able to switch between light and dark modes.

### Chat UI design and features

For each successful tool call that the model has generated, we will have a text message in the UI (looking exactly as in design):

- "Edited [file icon] [file name] [git diff lines]"
- "Viewed [file icon] [file name]"
- "Viewed [directory static icon] [directory]"

And other annotated tool calls.

If the model will fail, the user will be informed "The model has failed a tool call."

#### Adding context

The users should know what they are prompting with and should visually display the component they are passing as a context when selecting a prompt UI. When a user has clicked a part, a piece in a simulation, the part will be added to the context, above the UI. Please see [### Steerability](/kitty-specs/desired_architecture.md#steerability) section in the main spec for what can be added.
However, in the UI, it will be shown on the top of the chat UI.

By holding Ctrl, users can select multiple items, and each will have a card - parts, code items, and others.
On top of each "context" card, the users will be able delete it from context by pressing a cross on top of them (top right corner).

Frontend will only send a set of elements that are selected, and won't actually concatenate values to prompt; the backend will handle the prompting.

### Code viewer

The core workflow of a user is to view code and markdown plans that the system produces. Use a code viewer to display, lint (color), the code.

Requirements:

1. File tree,
2. Coloring (linting, meaning - markdown headings are linted, python syntax is colored, etc),
3. Line numbers.
4. Ideally, when the user selects a line, UI (see "#### Adding context" section), will add a line of the file that is selected, or a set of lines.

### Icons

Icons are from vscode, colorful. E.g. Python is blue and yellow, YAML has standard icons, etc.

## Collecting feedback from users

Users are to be able to submit thumbs up/down on model outputs, just as they would in common "chat" LLM UIs.

The users will be able to rate the agent when the model ends its output, and not at each message.

The feedback should be accompanied with a modal containing:

1. A way to edit the feedback (recall thumbs up or down)
2. A textbox explaining the feedback
3. A set of common feedback topics - what went wrong (misinterpretation, doesn't follow instructions, etc).

Ideally, the model would have a "selector"

## Dependencies

We will use Vite with React.

Our schema will be generated automatically via typescript type generation, updated via git hooks; dependant on OpenAPI schema from controller.
