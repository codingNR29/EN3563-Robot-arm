"""Use the adaptive gripper from the URDF, with an idealized grasp constraint."""
import xml.etree.ElementTree as ET


def task_description(source, description_share, controllers):
    root = ET.fromstring(source)
    # Fixed joints are normally lumped into link_6_t by URDF -> SDF conversion.
    root.append(ET.fromstring('''<gazebo><plugin filename="gz-sim-detachable-joint-system"
      name="gz::sim::systems::DetachableJoint"><parent_link>link_6_t</parent_link>
      <child_model>box_b</child_model><child_link>link</child_link>
      <attach_topic>/gp7/grasp/attach</attach_topic><detach_topic>/gp7/grasp/detach</detach_topic>
      <output_topic>/gp7/grasp/state</output_topic></plugin></gazebo>'''))
    text = ET.tostring(root, encoding='unicode')
    return text.replace('package://gp7_description/', description_share + '/').replace(
        '$(find gp7_gazebo)/config/ros2_controllers.yaml', controllers)
