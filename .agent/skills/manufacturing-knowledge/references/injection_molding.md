# Injection Molding Cost Model & Technical Reference

The Injection Molding Workbench simulates high-volume plastic production (typically ABS). It favors high quantities where large tooling investments are amortized.

## 1. Cost Components

### 1.1. Tooling Cost (Fixed)
- **Base Mold Cost**: ~$5,000.00.
- **Complexity Factor**: Increases based on the surface area of the part ($0.50 per cm^2$).
- **Discounting**: If the same part design is reused, tooling cost is discounted by **90%** (reflecting reusing the same physical mold).

### 1.2. Material Cost (Per Unit)
- **Formula**: $PartVolume_{cm^3} \times Density_{g/cm^3} / 1000 \times Price/kg$.
- **Optimization**: Direct correlation with part volume.

### 1.3. Cycle Cost (Per Unit)
The cycle time is the bottleneck of the process, determined by:
- **Injection Time**: $Volume / InjectionRate$ (Default $10 \, cm^3/s$).
- **Cooling Time**: The most critical factor. Proportional to the **square of the maximum wall thickness**.
  - $CoolingTime_s = CoolingFactor \times (MaxThickness_{mm})^2$.
  - Default Cooling Factor: $2.0 \, s/mm^2$ (for ABS).
- **Cycle Time**: $max(InjectionTime, CoolingTime)$.
- **Formula**: $CycleTime_s \times (MachineHourlyRate / 3600)$.

## 2. DFM Constraints

### 2.1. Draft Angles
- **Requirement**: All faces parallel to the pull direction (Z-axis) must have a draft angle.
- **Threshold**: Minimum **2.0 degrees**.
- **Violation**: Zero-draft faces will be rejected as they cannot be ejected from the mold.

### 2.2. Undercuts
- **2-Part Mold**: A face is an undercut only if it is occluded from **BOTH** +Z and -Z directions.
- **Constraint**: Simple molds cannot handle "trapped" geometry. If it can be reached from either the "A" or "B" side of the mold, it is valid.

### 2.3. Wall Thickness
- **Range**: Must stay between **1.0mm and 4.0mm**.
- **Reasoning**: < 1.0mm leads to short shots (incomplete fill); > 4.0mm leads to massive cooling times and sink marks.

## 3. Pricing Explanation Logic
The system provides this feedback to the agent:
> "Tooling cost ($T) is driven by surface area. Unit cost is driven by material volume and cooling time. Max thickness (M mm) dictates cooling time (C s)."
