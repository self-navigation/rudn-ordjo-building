from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xml.etree.ElementTree as ET
import os

def exclude_part(sdf_content: str, floor_num: int, cutoff_floor: int, part: str) -> str:
    if int(floor_num) >= cutoff_floor:
        root = ET.fromstring(sdf_content)
        model = root.find("model")
        base_link = model.find("./link[@name='base_link']")

        print(f"./visual[@name='{part}'_visual']")
        visual = base_link.find(f"./visual[@name='{part}_visual']")
        if visual is not None:
            base_link.remove(visual)
        collision = base_link.find(f"./collision[@name='{part}_collision']")
        if collision is not None:
            base_link.remove(collision)

        sdf_content = ET.tostring(root, encoding="unicode")
        return sdf_content

def generate_sdf(context, *args, **kwargs):
    floor_num = context.launch_configurations["floor_number"]

    package_dir = get_package_share_directory("rudn_ordjo_building")
    template_path = os.path.join(package_dir, "models", "model_template.sdf")
    with open(template_path, "r") as f:
        sdf_template = f.read()
    
    sdf_content = sdf_template.replace("{floor_num}", floor_num)

    sdf_content = exclude_part(sdf_content, floor_num, 4, "center")
    sdf_content = exclude_part(sdf_content, floor_num, 6, "right")

    sdf_content = sdf_content.replace("package://rudn_ordjo_building/", f"file://{package_dir}/")
    
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            f"rudn_ordjo_building_floor_{floor_num}",
            "-string",
            sdf_content,
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "0",
        ],
        output="screen"
    )
    
    return [spawn_entity]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "floor_number",
            default_value="3",
            description="Floor number to spawn (e.g., 3 for 3rd floor)"
        ),
        
        OpaqueFunction(function=generate_sdf)
    ])
