# CNC Milling Cost Model & Technical Reference

The CNC Workbench simulates a 3-axis milling process (typically Aluminum 6061). Costs are divided into fixed setup costs and variable per-unit costs.

## 1. Cost Components

### 1.1. Setup Cost (Fixed)
- **Base Rate**: Typically equal to 1 hour of machine time (e.g., $80.00).
- **Discounting**: If the same part design (determined by geometry hash) is reused in an assembly, subsequent setup costs are discounted by **50%** because the CAM program and fixturing can be reused.

### 1.2. Material Cost (Per Unit)
- **Stock Analysis**: The system assumes a rectangular stock block that exactly fits the part's bounding box.
- **Stock Volume**: $X_{mm} \times Y_{mm} \times Z_{mm} / 1000$ to get $cm^3$.
- **Formula**: $StockMass_{kg} \times MaterialPrice_{/kg}$.
- **Optimization**: Reducing the overall bounding box size (even if volume remains same) can reduce material cost.

### 1.3. Run Cost (Per Unit)
- **Removed Volume**: Calculated as $StockVolume - PartVolume$.
- **Machining Time**: Driven by the Material Removal Rate (MRR). 
  - $Time_{min} = RemovedVolume_{mm^3} / MRR$.
  - Default MRR: $1000 \, mm^3/min$.
- **Formula**: $(Time_{min} / 60) \times MachineHourlyRate$.
- **Optimization**: Designs that "hollow out" a block from the outside (increasing removed volume) actually *increase* the run cost, even if they save material weight.

## 2. DFM Constraints

### 2.1. Tool Access
- **3-Axis Assumption**: All machined faces must be visible from the +Z approach vector.
- **Undercuts**: Any face occluded from +Z is flagged as a violation.

### 2.2. Internal Radii
- **Tool Diameter**: Standard tool radius is **3.0mm**.
- **Internal Corners**: Internal vertical corners must have a radius $\ge 3.0mm$. It is recommended to use **3.1mm** to allow tool clearance and prevent chatter.

## 3. Pricing Explanation Logic
When analyzing CNC costs, the system reports:
> "Cost is driven by material volume and machining time. Stock size ([X, Y, Z]) determines material cost. Removed volume (V cm3) determines machining time (T min). Setup cost ($S) is fixed per part design."
