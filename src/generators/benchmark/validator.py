import os
import shutil
import tempfile
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from src.generators.benchmark.types import ValidationReport


def validate_mjcf(xml_string: str, asset_dir: str = None) -> ValidationReport:
    """
    Validates a MuJoCo XML string by loading it and running a short simulation
    to check for stability (no NaNs, no excessive velocities).
    """
    try:
        # Strip meshdir from XML to avoid absolute path issues during validation
        # We will use a temporary directory if asset_dir is provided
        root = ET.fromstring(xml_string)
        compiler = root.find("compiler")
        if compiler is not None and "meshdir" in compiler.attrib:
            del compiler.attrib["meshdir"]
        
        # Also, if we don't have an asset_dir, we might want to strip all <mesh> assets 
        # or replace them with primitives to allow basic physics validation if possible.
        # But for now, we assume if meshes are present, asset_dir MUST be provided or they must be in cwd.
        
        processed_xml = ET.tostring(root, encoding="unicode")

        # Load the model
        if asset_dir:
            with tempfile.TemporaryDirectory() as tmp_dir:
                # Copy assets to tmp_dir
                if os.path.exists(asset_dir):
                    for f in os.listdir(asset_dir):
                        shutil.copy(os.path.join(asset_dir, f), tmp_dir)
                
                xml_path = os.path.join(tmp_dir, "scene.xml")
                with open(xml_path, "w") as f:
                    f.write(processed_xml)
                model = mujoco.MjModel.from_xml_path(xml_path)
        else:
            model = mujoco.MjModel.from_xml_string(processed_xml)
            
        data = mujoco.MjData(model)

        # Simulation parameters
        duration = 1.0  # seconds
        timestep = model.opt.timestep if model.opt.timestep > 0 else 0.002
        steps = int(duration / timestep)
        max_velocity_threshold = 100.0  # m/s

        max_energy = 0.0

        # Step the simulation
        for _ in range(steps):
            mujoco.mj_step(model, data)

            # Check for NaNs in state
            if np.isnan(data.qpos).any() or np.isnan(data.qvel).any():
                return ValidationReport(
                    is_valid=False,
                    error_message="Simulation diverged (NaN detected)",
                    sim_duration=data.time,
                    max_energy=max_energy,
                )

            # Check for excessive velocities
            current_max_vel = np.max(np.abs(data.qvel)) if data.qvel.size > 0 else 0.0
            if current_max_vel > max_velocity_threshold:
                return ValidationReport(
                    is_valid=False,
                    error_message=f"Velocity exceeded threshold: {current_max_vel:.2f} > {max_velocity_threshold}",
                    sim_duration=data.time,
                    max_energy=max_energy,
                )

            # Track max energy (kinetic + potential)
            # Just a simple check, not strict validation criteria yet
            mujoco.mj_energyPos(model, data)
            mujoco.mj_energyVel(model, data)
            energy = data.energy[0] + data.energy[1]
            max_energy = max(max_energy, energy)

        return ValidationReport(
            is_valid=True,
            error_message=None,
            sim_duration=data.time,
            max_energy=max_energy,
        )

    except Exception as e:
        return ValidationReport(
            is_valid=False,
            error_message=f"XML Loading/Runtime Error: {e!s}",
            sim_duration=0.0,
            max_energy=0.0,
        )
