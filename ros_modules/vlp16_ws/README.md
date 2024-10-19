# ROS2

Contents

1. Install ROS2
2. Setup Environment and Workspace
3. ROS Tools
4. Packages
5. Nodes
6. Topics
7. Services
8. Interfaces
9. Parameters
10. Launch Files
11. Turtlesim


## Install ROS2

## Setup Environment and Workspace

## ROS Tools

### ros2-cli

```
ros2 <cmd>
ros2 <cmd> -h

<cmd>
-------------------------------------------------
- action     Various action related sub-commands
- bag        Various rosbag related sub-commands
- component  Various component related sub-commands
- daemon     Various daemon related sub-commands
- doctor     Check ROS setup and other potential issues
- [x] interface  Show information about ROS interfaces
- launch     Run a launch file
- lifecycle  Various lifecycle related sub-commands
- multicast  Various multicast related sub-commands
- [x] node       Various node related sub-commands
- [x] param      Various param related sub-commands
- [x] pkg        Various package related sub-commands
- [x] run        Run a package specific executable
- security   Various security related sub-commands
- [x] service    Various service related sub-commands
- [x] topic      Various topic related sub-commands
- wtf        Use `wtf` as alias to `doctor`
-------------------------------------------------
```

### rqt 

Node graph:
- `rqt_graph`


## Packages

**Create**
```bash
# python
ros2 pkg create <pkg_name> --build-type ament_python --dependencies rclpy
# c++ 
ros2 pkg create <pkg_name> --build-type ament_cmake --dependencies rclcpp
```

**Build**

```bash
colcon build --packages-select <pkg_name>
# python build with symlink during dev.
colcon build --packages-select <pkg_name> --symlink-install
```

## Nodes
- Subprogram responsible for only one task
- Combined into graph
- Communicate through Topics, Services, and Parameters
- Fault tollerant and language-agnostic [ `python <--> C++`]
- A node can have many pub/sub for different topics

**setup node**
- `reader_node` to setup.py
- `converter_node` to CMakeLists.txt [create xec & install pkg into ws]
- rebuild package | `colcon build`

**run exec-s**

python
- `./install/pcap2pcd_py/lib/pcap2pcd_py/reader_node`
- `ros2 run pcap2pcd_py reader_node`

cpp
- `./install/pcd_converter_cpp/lib/pcd_converter_cpp/convert_node`
- `ros2 run pcd_converter_cpp convert_node`


**node info**

- `ros2 node list`
- `ros2 node info <node_name>`

rename node at runtime:
- `ros2 run pcap2pcd_py reader_node --ros-args --remap __node:=reader`


## Topics

- A topic is a named bus over which nodes exchange messages.
- Unidirectional data stream [ pub/sub ] ( anonymous )
- A topic has a message type [ all pub/sub must used this type ]


**Topic Info**

- `ros2 topic ...` 
- `ros2 topic list`
- `ros2 topic info <topic_name>`
- `ros2 topic echo /topic_name` : published info


rename topic at runtime:
- `ros2 run <pkg_name> <node_name> --ros-args --remap __node:=<new_name> -r <topic_name>:=<new_topic_name>`


### Publisher

```python
Node.create_publisher(msg_type, <topic_name>, <qos_profile>)
```

tmp_publisher
```bash
ros2 topic pub -r 10 /<topic_name> example_interfaces/msg/String "{data: 'temporary pub at 10 Hz'}"
```

### Subscriber

```python
Node.create_subscription(msg_type, topic, callback, qos_profile)
```

### Message

**msg_type**
e.g. `ros2 interface show example_interfaces/msg/...`


## Services

- A client/server system
- Sync/Async
- One msg_type for Request and One msg_type for Response
- A service server exist once, but can have many clients

e.g. `ros2 interface show example_interfaces/srv/...`


**Service Info**

- `ros2 service ...` 
- `ros2 service list`
- `ros2 service type <srv_name>`
- `ros2 service call <srv_name> <srv_type>, "{request params}"`

**Create a Service**

```python
Node.create_service(srv_type, srv_name, callback)
```
**Create a Client**

```python
Node.create_client(srv_type, srv_name)
```
**Rename service at runtime:**
- `ros2 run <pkg_name> <node_name> --ros-args --remap __node:=<new_name> -r <srv_name>:=<new_srv_name>`


## Interfaces

- Interfaces are the messages and services used with topics and services

- `ros2 interface list`
- `ros2 interface package sensor_msgs`
  
### Custom msg

As a different package :

`ros2 pkg create <pkg_interfaces>` then rm -> src/ & include/ then mkdir -> msg

to -> package.xml
```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

to -> CMakelist.txt
```cmake
find_package(rosidl_default_generators REQUIRED)
```

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
)
```

build : `colcon build --packages-select point_cloud_interfaces`

show : `ros2 interface show point_cloud_interfaces/msg/HardwareStatus`

run : `ros2 run <pkg> hw_status_pub `
  - `ros2 topic info /hw_status`
  - `ros2 topic echo /hw_status`

### Custom srv

mkdir -> srv

to -> CMakelist.txt
```cmake
find_package(rosidl_default_generators REQUIRED)
```

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HardwareStatus.msg"
  "srv/ComputeRectangleArea.srv"
)
```

build : `colcon build --packages-select point_cloud_interfaces`

show : `ros2 interface show point_cloud_interfaces/srv/ComputeRectangleArea `

## Parameters

- Configs for the nodes, value set at run-time
- It is specific to a node
- Param-types( bool, int, list, ... )

`ros2 param ...`
`ros2 param list`
`ros2 param get /<node_name> <param_name>`

**Declare parameters**

```python
Node.declare_parameter("param_name")
```

**Get parameters**

```python
Node.get_parameter("param_name")
```

**set parameters at runtime**
`ros2 run <pkg_name> <node_name> --ros-args -p <param_name>:=<value> -p ...`

## Launch Files

## Turtlesim

install:
- `sudo apt install ros-foxy-turtlesim`
- `ros2 run turtlesim ... `