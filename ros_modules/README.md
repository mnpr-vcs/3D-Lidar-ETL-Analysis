# Robot Operating System

`Open Source Framework, containing tools, libs, etc. (C++/ Python)`


## Installation

- http://wiki.ros.org/Installation/Ubuntu

## Fundamentals

**roscore**

- Collection of nodes and programs
- Sets up all communication
- Every Project needs running roscore

**Nodes**

- A single executable file performing some task
- Communicate with other node through services and topics

**Topics**

- Used to communicate between nodes
- Publisher and Subscriber
- Only one publisher, many subscribers

**Message**

- Describes the messages published to topics

***

### Writing a Publisher/Subscriber :

```
workspace_folder/
    - src/
        - <pkg-name>
            - src/
                - ...
            - msg/
                - position.msg
            - scripts/
                - publisher_node.py
                - subscriber_node.py
            - CMakeLists.txt
            - package.xml
```

- Inside workspace_folder/ : bash$> `catkin_make`
  
  - For python3: `catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3`

- Inside workspace_folder/src/ Create package: ` catkin_create_pkg <pkg-name> rospy std_msgs`

- Run `roscore` in terminal_1

- Run `source devel/setup.zsh` & `rosrun <pkg-name> publisher_node.py` in terminal_2

- Run `source devel/setup.zsh` & `rosrun <pkg-name> subscriber_node.py` in terminal 3

- Run `rostopic list` to list published topics

- Run `rostopic echo /<topic_name>` to get the message in the topic

### Writing Custom Messages
