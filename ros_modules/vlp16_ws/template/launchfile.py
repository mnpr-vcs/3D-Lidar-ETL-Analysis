from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    ld = LaunchDescription()
    
    pcap_node = Node(
        package='<pkg_name>',
        executable='<xec_name>'
    )
    
    pc2_node = Node(
        package='<pkg_name>',
        executable='<xec_name>'
    )
    
    ld.add_action(pcap_node)
    ld.add_action(pc2_node)
    
    return ld