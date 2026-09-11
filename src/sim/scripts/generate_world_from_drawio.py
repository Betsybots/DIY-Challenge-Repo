import math
import sys
import xml.etree.ElementTree as ET

# Input and output files
drawio_file = "/home/matrix/Downloads/layout1.drawio"
gazebo_world_file = "/home/matrix/autonomous-race-car/src/sim/worlds/layout2.world"

# Parameters for Gazebo model
MODEL_URI = "../models/straw_bale/model.sdf"
MODEL_Z = 0.25
MODEL_ROLL = 0
MODEL_PITCH = 0

# Input arguments
if len(sys.argv) < 3:
    print(
        "Usage: python generate_world_from_drawio.py <input_drawio_file> <output_gazebo_world_file>"
    )
    sys.exit(1)

drawio_file = sys.argv[1]
gazebo_world_file = sys.argv[2]

# Parse the draw.io file
tree = ET.parse(drawio_file)
root = tree.getroot()

namespace = {"mx": "http://www.w3.org/1999/xhtml"}


def extract_grid_size(root):
    # Assuming the grid size is defined in the diagram's attributes
    # If not available, we set a default grid size of 10 (which can be adjusted)
    for element in root.findall(".//mxGraphModel/root", namespace):
        if "gridSize" in element.attrib:
            return float(element.attrib["gridSize"])
    return 10.0  # Default grid size


def extract_green_rectangles(root, scale):
    rectangles = []
    for element in root.findall(".//mxCell", namespace):
        if (
            "style" in element.attrib and "fillColor=#008a00" in element.attrib["style"]
        ):  # Green color in hex in style attribute
            geometry = element.find("mxGeometry", namespace)
            if geometry is not None:
                # Extract position, dimensions, and rotation
                x = float(geometry.attrib.get("x", 0)) * scale
                y = float(geometry.attrib.get("y", 0)) * scale
                width = float(geometry.attrib.get("width", 1)) * scale
                height = float(geometry.attrib.get("height", 1)) * scale
                rotation = 0
                if "rotation" in element.attrib["style"]:
                    style_parts = element.attrib["style"].split(";")
                    for part in style_parts:
                        if part.startswith("rotation="):
                            rotation = float(part.split("=")[1])
                yaw = -math.radians(rotation)  # Adjust yaw to account for Y-axis inversion
                rectangles.append((x, y, width, height, yaw))
    return rectangles


def convert_to_gazebo(rectangles):
    include_tags = []
    min_x = min(rect[0] for rect in rectangles)
    min_y = min(rect[1] for rect in rectangles)
    max_x = max(rect[0] + rect[2] for rect in rectangles)
    max_y = max(rect[1] + rect[3] for rect in rectangles)

    # Calculate the center offset to set (0,0) at the center of the map
    offset_x = (min_x + max_x) / 2
    offset_y = (min_y + max_y) / 2

    for i, rect in enumerate(rectangles):
        x, y, width, height, yaw = rect
        # Convert draw.io coordinates to Gazebo world coordinates (considering each cell is 1m x 1m)
        center_x = (x + (width / 2)) - offset_x
        center_y = -(
            (y + (height / 2)) - offset_y
        )  # Invert y-axis to match Gazebo coordinate system
        # Scale dimensions to meters (1 unit in draw.io = 1 meter in Gazebo)
        include_tag = f"""
        <include>
            <name>model_{i}</name>
            <uri>{MODEL_URI}</uri>
            <pose>{center_x} {center_y} {MODEL_Z} {MODEL_ROLL} {MODEL_PITCH} {yaw}</pose>
        </include>
        """
        include_tags.append(include_tag)
    return include_tags


# Extract grid size from the draw.io file
grid_size = extract_grid_size(root)
# Compute scale ratio: 1 meter in Gazebo corresponds to grid_size in draw.io
scale = 0.25 / grid_size

# Extract green rectangles from the draw.io file
rectangles = extract_green_rectangles(root, scale)

# Convert the rectangles to Gazebo world include tags
include_tags = convert_to_gazebo(rectangles)

# Write the Gazebo world file
with open(gazebo_world_file, "w") as world_file:
    world_file.write('<?xml version="1.0" ?>\n')
    world_file.write('<sdf version="1.6">\n')
    world_file.write('  <world name="default">\n')
    for tag in include_tags:
        world_file.write(tag)
    world_file.write("  </world>\n")
    world_file.write("</sdf>\n")

print(f"Gazebo world file generated: {gazebo_world_file}")
