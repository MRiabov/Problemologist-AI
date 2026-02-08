# Desired Architecture: WP3 - Electronics

## Objective of the system

To enable the design and verification of electromechanical systems. The agent shall design not just the physical parts (enclosures, mounts), but also the electronics (PCBs, wiring, motors, sensors) that power and control them.

### Goals

1. **PCB Design**: Generate valid KiCad PCB files (netlist + connectivity + 3D layout).
2. **Electromechanical Integration**: Verify that the PCB fits in the enclosure, mounting holes align, and connectors are accessible.
3. **Circuit Logic Verification**: Verify simple logic via emulation (e.g., "Motor spins when button pressed").
4. **BOM Costing**: Calculate the total cost of the electronic Bill of Materials.

## The Electronics Engineer Agent

We introduce a specialized sub-agent, the **Electronics Engineer**. This agent is distinct from the Mechanical Engineer because it operates in a different domain (Circuit Theory vs Solid Mechanics) and uses different tools.

### Responsibilities

1. **Component Selection**: Choosing parts from a library that meet requirements (Voltage, Current, Footprint).
2. **Schematic/Netlist Definition**: Defining how parts are connected electrically.
3. **PCB Layout**: Placing components on a 2D board surface to satisfy physical constraints.
4. **Export**: Generating manufacturing files (Gerbers, Drill files) and simulation files (STEP models).

### Interaction with Mechanical Engineer

The interaction is a bidirectional negotiation:

* **Scenario A: "Mechanical First"**
    1. **Mech**: "I have designed a robot arm. I have a volume of `50x50x20mm` available for the controller. Here is the STEP file of the enclosure."
    2. **Elec**: "I will design a PCB to fit. I need to place a microcontroller and 2 motor drivers."
    3. **Elec**: "Done. Here is the STEP file of the PCB."
    4. **Mech**: "Checking fit... Collision detected. The capacitor C1 is too tall. Please move it or choose a smaller one."

* **Scenario B: "Electrical First"**
    1. **Elec**: "I need a PCB with these dimensions to support high-current traces."
    2. **Mech**: "I will build the enclosure around it."

## Tools & Workflows

### Electronic Design Automation (EDA)

We use **KiCad 8+** as the core technology.

* **Why**: Open source, scriptable via Python, huge community, standard file formats (`.kicad_pcb`, `.kicad_sch`).
* **Pipeline**:
    1. **SKiDL**: We use `skidl` (Python library) to define the Netlist programmatically.
    2. **KiCad PCBNew**: We use the Python API of `pcbnew` to place components and route traces.

### The Headless EDA Service (Microservice)

To avoid installing 2GB of KiCad binaries in the main agent container:

* **Service**: `eda-worker` container.
* **API**: `POST /render-pcb` accepts a Netlist/Layout script and returns a `.kicad_pcb` + `.step` file.
* **Isolation**: Runs a virtual X server (`Xvfb`) internally if any GUI tools (like `freerouting`) need a display handle.

### Component Library (The "Parts Database")

The agent cannot just "hallucinate" a resistor. It must pick a real part with real dimensions.
We maintain a curated, local SQLite database of "Approved Components".

#### Schema

```sql
CREATE TABLE components (
    id TEXT PRIMARY KEY, -- 'R-0805-10k'
    category TEXT, -- 'Resistor', 'MCU', 'Connector'
    value TEXT, -- '10k', 'STM32F103'
    footprint TEXT, -- 'Resistor_SMD:R_0805_2012Metric'
    symbol TEXT, -- 'Device:R'
    price_usd REAL,
    datasheet_url TEXT,
    step_file_path TEXT, -- Path to 3D model for collision check
    snapshot_version INT DEFAULT 1 -- Increment on update
);
```

### Reproducibility (Library Snapshotting)

Component availability changes (prices change, parts go EOL).

* **Snapshotting**: When a design is finalized, we freeze the `component` rows used.
* **Cache**: The `eda-worker` caches the specific `step_file_path` version used at design time, so a re-render 1 year later doesn't break because a model file changed.

#### Tool: `query_library(params)`

* *Input*: `{"category": "Capacitor", "min_voltage": "16V", "max_height": "5mm"}`
* *Output*: `[{"id": "C-1206-10uF", "price": 0.05, ...}, ...]`

### Simulation & Verification

#### 1. Logic Emulation

We do not simulate analog signal integrity (too complex for MVP). We simulate **Digital Logic**.

* **Tool**: `simulate_firmware(code, circuit_netlist)`
* **Backend**: `QEMU` (for MCU) + `SimulIDE` (for peripheral logic).
* **Test**: "If `GPIOA.Pin1` goes HIGH -> Does `MotorDriver.Input1` receive HIGH?"

#### 2. Mechanical Fit (The "Fit Check")

This is the most critical step for the Problemologist.

* **Tool**: `check_pcb_fit(pcb_step_file, enclosure_step_file)`
* **Backend**: `build123d` boolean operations.
  * Load Assembly.
  * Check `PCB.intersect(Enclosure).volume > 0`.
  * Check `MountingHoles.aligned()`.

## Detailed Workflow Example: "Blinking LED"

1. **Planner**: "Task: Create a device that blinks an LED."
2. **Elec Agent**:
    * `query_library(category="MCU")` -> Selects `ATTINY85`.
    * `query_library(category="LED")` -> Selects `LED-5mm-Red`.
    * `query_library(category="Battery")` -> Selects `CR2032-Holder`.
3. **Elec Agent (Coding)**:

    ```python
    from skidl import *
    from my_lib import *

    mcu = Part("my_lib", "ATTINY85")
    led = Part("my_lib", "LED-5mm-Red")
    res = Part("my_lib", "R-0805-330R")
    bat = Part("my_lib", "CR2032-Holder")

    mcu['VCC'] += bat['+']
    mcu['GND'] += bat['-']
    mcu['PB0'] += led['A']
    led['K'] += res[1]
    res[2] += mcu['GND']

    generate_netlist()
    ```

4. **Elec Agent (Layout)**:
    * Generates `blink.kicad_pcb` from netlist.
    * Places components on a 30x30mm board.
    * Runs Auto-router.
    * Exports `blink.step`.
5. **Mech Agent**:
    * Imports `blink.step`.
    * Designs a box around it with holes for the LED and Battery access.
    * Checks for collisions.
6. **Verifier**:
    * Compiles dummy firmware (Blink).
    * Runs logic simulation (Does the pin toggle?).
    * Checks final assembly mass and cost.

## Tech Stack

* **KiCad 8+**: The standard EDA suite.
* **SKiDL**: Python-to-Netlist compiler. Allows "Circuit as Code".
* **Kicad-automation-scripts**: Docker container with KiCad and automation tools installed.
* **Freerouting**: Java-based auto-router (headless) for simple boards.
* **QEMU / Renode**: For MCU emulation.
* **Build123d**: For importing STEP files and checking mechanical fit.

## Challenges & Mitigations

* **Auto-routing Quality**: Auto-routers are bad at complex signals.
  * *Mitigation*: We stick to simple boards (2-layer, low frequency) where auto-routers suffice.
* **Component 3D Models**: Many library parts lack 3D models.
  * *Mitigation*: We prioritize a small "Golden Library" of parts that *do* have STEP files. If a part is missing 3D, we verify using bounding boxes (derived from footprint Courtyard layer).

## Future Work

1. **Wire Harness Design**: Generating 3D cables between boards (very hard in standard CAD).
2. **Thermal Simulation**: Simulating heat dissipation from the PCB to the enclosure (linked to WP6).
3. **Gerber Analysis**: Validating manufacturing files against specific Fab House capabilities (e.g., JLCPCB rules).

## Supply Chain & BOM Management

The Electronics Agent must understand that parts are physical goods with lead times, not just symbols.

### The "Stock Check"

Before finalizing a design, the agent performs a `check_availability(bom)` call.

* **API**: Integration with DigiKey/Mouser API (or a cached snapshot).
* **Logic**:
  * If `Stock < 100`, mark as **Risk**.
  * If `Stock == 0`, triggering **Part Swap**.

### Part Swap Workflow

1. **Alert**: "C12 (10uF 16V) is out of stock."
2. **Search**: Agent searches for `Capacitor, 10uF, >=16V, Footprint=0603`.
3. **Selection**: Agent picks `Alternative_Part_ID`.
4. **Update**: Agent updates the Schema: `C12.part_id = Alternative_Part_ID`.
   *(Note: Netlist and Layout usually don't change if footprint is identical, but cost might).*

### Data Export (The Manufacturing Package)

The agent generates a zip file `production_v1.zip` containing:

1. `gerbers/` (Standard RS-274X)
2. `drill/` (Excellon)
3. `pick_and_place.csv` (Centroids for assembly)
4. `bom.csv` (Mfr Part Numbers)
5. `assembly_drawings.pdf` (Visual guide for humans)

This package is technically ready to upload to JLCPCB/PCBWay.
