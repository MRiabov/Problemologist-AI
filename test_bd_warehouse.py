try:
    from bd_warehouse.fastener import SocketHeadCapScrew
    from build123d import Location, Part, Box, RigidJoint

    screw = SocketHeadCapScrew(size="M3-0.5", length=10)
    print("Created M3-0.5 screw")

    # List attributes
    print(dir(screw))

    # Check for diameter properties
    if hasattr(screw, "shank_diameter"):
        print(f"Shank diameter: {screw.shank_diameter}")
    if hasattr(screw, "head_diameter"):
        print(f"Head diameter: {screw.head_diameter}")
    if hasattr(screw, "head_height"):
        print(f"Head height: {screw.head_height}")

    # Check if there is clearance hole data
    if hasattr(screw, "clearance_hole_diameters"):
        print(f"Clearance hole diameters: {screw.clearance_hole_diameters}")

except Exception as e:
    print(f"Error: {e}")
    import traceback

    traceback.print_exc()
