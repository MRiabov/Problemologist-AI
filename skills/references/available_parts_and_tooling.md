# Manufacturing Capabilities & Standard Components Catalog

This document details the available manufacturing processes, materials, and off-the-shelf (COTS) components available for engineering design tasks. All designs must adhere to the constraints specified herein.

## 1. Custom Fabrication Capabilities

Our facility offers the following manufacturing processes for custom component production.

### 1.1 CNC Machining

Subtractive manufacturing for high-precision metal and plastic parts.

* **Materials**:
  * **Aluminum 6061** (`aluminum_6061`): General purpose alloy. Good strength-to-weight ratio.
    * Density: 2.7 g/cm³
    * Yield Strength: 276 MPa
    * Cost: ~$6.00/kg (plus machining time)
* **Process Constraints**:
  * Min Tool Radius: 3.0mm (avoid sharp internal corners).
  * Standard Axis: 3-Axis milling.
* **Cost Drivers**: Setup time + Machining time (Material Removal Rate ~1000 mm³/min).

### 1.2 Injection Molding

High-volume production for plastic components.

* **Materials**:
  * **ABS Plastic** (`abs`): Durable, impact-resistant thermoplastic.
    * Density: 1.04 g/cm³
    * Yield Strength: 40 MPa
    * Cost: ~$2.50/kg
* **Process Information**:
  * Min Wall Thickness: 1.0mm
  * Max Wall Thickness: 4.0mm
  * Draft Angle: min 2.0° required for ejection.
* **Cost Drivers**: High initial mold cost ($5000+), low unit cost. Best for mass production.

### 1.3 Rapid Prototyping (3D Printing)

Additive manufacturing for complex geometries and rigorous testing.

* **Suitability**: Low-stress parts, complex internal geometries, validation models.
* **Materials**: Various thermoplastics (PLA, PETG, Nylon).

---

## 2. Standard Components Catalog (COTS)

Certified commercial off-the-shelf components available for assembly.

### 2.1 Electromechanical Actuators (Motors)

| Model | Type | Torque (max) | Speed (max) | Weight/Cost | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **SG90 Micro** | Servo | 0.18 Nm | 10.5 rad/s | Low / $3.50 | Lightweight applications, linkages. |
| **MG996R** | Servo | 1.1 Nm | 6.3 rad/s | Med / $12.00 | Standard metal-gear high-torque servo. |
| **Dynamixel AX12A** | Smart Servo | 1.5 Nm | 11.9 rad/s | High / $45.00 | Precision robotic actuator with feedback. |
| **NEMA-17** | Stepper | 0.45 Nm | 31.4 rad/s | Med / $15.00 | Precision position control, high holding torque. |

* **Interface**: Controlled via standardized signal drivers (`constant`, `sinusoidal`, `waypoint` profiles).
* **Protection**: Integrated thermal overload protection (shutdown after >2s stalled condition).

### 2.2 Fastening Hardware

Standard metric fasteners for rigid assembly. Sourced from `bd-warehouse`.

* **Types**: Hex Bolts, Socket Head Cap Screws, Nuts, Washers.
* **Standard**: Metric (ISO).
* **Assembly Standard**:
  * **Rigid Joints**: Minimum 2 fasteners required per connection interface to prevent rotation.
  * **Through-Holes**: Must match fastener ISO tolerances.

---

## 3. Design & Validation Environment

### 3.1 Digital Twin Simulation

All designs undergo virtual validation before fabrication approval.

* **Physics Engine**: MuJoCo Rigid Body Dynamics.
* **Simulation Fidelity**:
  * Collision detection (0.05s intervals).
  * Material physics (Friction, Restitution) based on material selection.
  * Gravity & rigid body mechanics.
* **Validation Scope**:
  * Kinematic feasibility.
  * Structural stability (no disintegration under load).
  * Operational verification (task completion).

### 3.2 Virtual Metrology & Costing

* **Automated Costing**: Real-time estimation of unit cost based on selected material and process.
* **Geometric Validation**: Automated checks for:
  * Build Volume compliance (interference with Working Envelope).
  * Keep-Out Zone violations (interference with Restricted Areas).
* **Manufacturability Checks**: Automated Design-for-Manufacturing (DFM) feedback (e.g., wall thickness, tool access).

---

## 4. Operational Constraints

### 4.1 Working Envelope

* **Build Volume**: Defined per project (typically Axis-Aligned Bounding Box).
* **Restricted Areas**: "Keep-Out" zones defined by existing infrastructure or safety requirements.

### 4.2 Budgetary Limits

* **Target Unit Cost**: Defined per project scope.
* **Max Weight**: Defined per project scope.
