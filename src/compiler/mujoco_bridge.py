import os
import mujoco
import xml.etree.ElementTree as ET
from dataclasses import dataclass
import numpy as np


@dataclass
class SimResult:
    duration: float
    energy: float
    success: bool
    damage: float


class MujocoBridge:
    def __init__(self):
        # Assumes this file is in src/compiler/
        # and templates are in src/compiler/templates/
        self.template_dir = os.path.join(os.path.dirname(__file__), "templates")

    def load_template(self, template_name: str = "standard.xml") -> str:
        """Load a standard template and return its XML string content.

        Args:
            template_name: Name of the XML file in templates directory.

        Returns:
            The raw string content of the XML file.
        """
        path = os.path.join(self.template_dir, template_name)
        if not os.path.exists(path):
            raise FileNotFoundError(f"Template not found: {path}")

        with open(path, "r") as f:
            return f.read()

    def inject_design(
        self,
        xml_content: str,
        mesh_path: str,
        location: tuple[float, float, float] = (0, 0, 1),
    ) -> str:
        """Inject a mesh into the MJCF XML at the specified location.

        Args:
            xml_content: Base MJCF XML string.
            mesh_path: Absolute path to the mesh file (.stl, .obj, etc).
            location: (x, y, z) tuple for the body position.

        Returns:
            Modified MJCF XML string with the mesh injected as a free body.
        """
        root = ET.fromstring(xml_content)

        # 1. Add asset -> mesh
        asset = root.find("asset")
        if asset is None:
            asset = ET.Element("asset")
            worldbody_node = root.find("worldbody")
            if worldbody_node is not None:
                children = list(root)
                index = children.index(worldbody_node)
                root.insert(index, asset)
            else:
                root.append(asset)

        mesh_name = os.path.basename(mesh_path).replace(".", "_")

        # Avoid duplicate mesh entries
        existing_mesh = asset.find(f"./mesh[@name='{mesh_name}']")
        if existing_mesh is None:
            # We must map 'file' attribute to the absolute path
            ET.SubElement(asset, "mesh", file=mesh_path, name=mesh_name)

        # 2. Add worldbody -> body -> geom
        worldbody = root.find("worldbody")
        if worldbody is None:
            raise ValueError("Invalid MJCF: missing worldbody")

        body_name = "injected_object"
        loc_str = f"{location[0]} {location[1]} {location[2]}"

        body = ET.SubElement(worldbody, "body", name=body_name, pos=loc_str)
        ET.SubElement(body, "freejoint")
        ET.SubElement(body, "geom", type="mesh", mesh=mesh_name)

        return ET.tostring(root, encoding="unicode")

    def run_simulation(self, xml_string: str, duration: float = 5.0) -> SimResult:
        """Run the simulation headless and return metrics.

        Args:
            xml_string: The MJCF XML content.
            duration: Simulation logic time in seconds.

        Returns:
            SimResult object with metrics.
        """
        try:
            model = mujoco.MjModel.from_xml_string(xml_string)
        except Exception as e:
            print(f"MuJoCo Load Error: {e}")
            # Return failure result instead of crashing
            return SimResult(duration, energy=0.0, success=False, damage=100.0)

        data = mujoco.MjData(model)

        energy_acc = 0.0
        damage_acc = 0.0
        success = False

        try:
            while data.time < duration:
                mujoco.mj_step(model, data)

                # Energy Metric: Accumulate |power| = |force . velocity|
                if model.nu > 0:
                    power = np.dot(data.qfrc_actuator, data.qvel)
                    energy_acc += abs(power) * model.opt.timestep

                # Damage Metric: Monitor contacts (placeholder logic)
                # If we define specific logic later, we can iterate data.contact
                pass

        except Exception as e:
            print(f"Simulation Runtime Error: {e}")
            return SimResult(duration, energy=0.0, success=False, damage=100.0)

        # Check Success (Object Validity)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "injected_object")
        if body_id != -1:
            pos = data.xpos[body_id]
            # Success if position is valid (not NaN)
            if not np.any(np.isnan(pos)):
                success = True

        return SimResult(
            duration=duration, energy=energy_acc, success=success, damage=damage_acc
        )
