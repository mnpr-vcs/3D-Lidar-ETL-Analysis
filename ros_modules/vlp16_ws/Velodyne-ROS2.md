# Velodyne-ROS2 Packages

## velodyne_driver
The driver is responsible for taking the data from the Velodyne and combining it into one message per revolution.
To turn such raw data to pointcloud or laser scan 

- `velodyne_pointcloud`
- `velodyne_laserscan`

**Published Topic**
`/velodyne_packets` (`velodyne_msgs/VelodyneScan`)

**Parameters**

- `device_ip` (string) - The IP address that the Velodyne is on.  From the factory, this is 192.168.1.201.
- `gps_time` (bool) - Whether to use data capture time from a GPS, or from the local time.  Should only be set to True if a GPS is attached to the Velodyne.  False by default.
- `time_offset` (double) - An arbitrary "skew", in seconds, to add to the acquisition timestamp.  Defaults to 0.0.
- `enabled` (bool) - Whether the device should start-up enabled or not.  Defaults to True.
- `read_once` (bool) - Whether to only playback the data once (True) or continuously (False).  Only used in PCAP playback mode.  Defaults to False.
- `read_fast` (bool) - Whether to output the data as fast as possible, (True), or sleep the appropriate delay between packets (False).  Only used in PCAP playback mode.  Defaults to False.
- `repeat_delay` (double) - The time to wait between repeats in continuous playback mode.  Only used in PCAP playback mode.  Defaults to 0.0.
- `frame_id` (string) - The frame_id to use when constructing the header for the packet to be published.  Defaults to "velodyne".
- `model` (string) - The model number of the Velodyne attached.  This should be one of `64E`, `64E_S2`, `64E_S2.1`, `64E_S3`, `32E`, `32C`, or `VLP16`.  Defaults to `64E`.
- `rpm` (double) - The RPM that the Velodyne is configured for.  Note that this is descriptive, not prescriptive, so this should be set to match the value configured through the Velodyne web interface.
- `pcap` (string) - The PCAP playback file to use to playback data from.  Only used in PCAP playback mode.  Defaults to the empty string.
- `cut_angle` (double) - The azimuth angle at which to declare a single rotation complete.  If this is less than 0, then a fixed number of packets (device-dependent) is used per rotation.  This mostly works, but can vary because of variations in the hardware.  If a positive number <= 2*Pi, a rotation will be declared "complete" when the azimuth reported by the device reaches that value.  Defaults to 2*Pi.
- `port` (int) - The port on which to receive data from the Velodyne.  From the factory, the Velodyne is configured to publish data on 2368.  Defaults to 2368.


## velodyne_laserscan( pointcloud to laserscan )

This  package that takes pointcloud data as output by one of the velodyne_pointcloud nodes and converts it to a single laserscan.

**Published Topics**

- scan (sensor_msgs/LaserScan) - The laserscan that results from taking one line of the pointcloud.

**Subscribed Topics**

- velodyne_points (sensor_msgs/PointCloud2) - The pointcloud that results from the raw velodyne data.

**Parameters**

- ring (int) - The "ring" of the Velodyne to use for the single line. If less than 0, a default ring per device will be used. Defaults to -1.
- resolution (double) - The resolution in meters that each point provides. Defaults to 0.007.

## velodyne_pointcloud (raw to pointcloud)

This  package that takes raw velodyne data as output by the velodyne_driver node, and converts it into a sensor_msgs/PointCloud2 message.

There are two different nodes here:

- `velodyne_convert_node` - This node takes in the raw data and converts it to a pointcloud immediately. 
- `velodyne_transform_node` - This node takes the raw data and converts it only when a corresponding tf message with a compatible timestamp has arrived.

The topics and parameters for both nodes are identical.

**Published Topics**

- `velodyne_points` (`sensor_msgs/PointCloud2`) - The pointcloud that results from the raw velodyne data.

**Subscribed Topics**

- `velodyne_packets` (`velodyne_msgs/VelodyneScan`) - The raw velodyne packets coming from the velodyne_driver.

**Parameters**

- `calibration` (string) - The path to the calibration file for the particular device.  There are a set of default calibration files to start with in the "params" subdirectory in this package.  Defaults to the empty string.
- `min_range` (double) - The minimum range in meters that a point must be to be added to the resulting point cloud.  Points closer than this are discarded.  Must be between 0.1 and 10.0.  Defaults to 0.9.
- `max_range` (double) - The maximum range in meters that a point must be to be added to the resulting point cloud.  Points further away than this are discarded.  Must be between 0.1 and 200.0.  Defaults to 130.0.
- `view_direction` (double) - The point around the circumference of the device, in radians, to "center" the view.  Combined with `view_width`, this allows the node to generate a pointcloud only for the given width, centered at this point.  This can vastly reduce the CPU requirements of the node.  Must be between -Pi and Pi, where 0 is straight ahead from the device.  Defaults to 0.0.
- `view_width` (double) - The width, in radians, of the view to generate for the resulting pointcloud.  Combined with `view_direction`, this allows the node to generate a pointcloud only for the given width, centered at the `view_direction` point.  This can vastly reduce the CPU requirements of the node.  Must be between 0 and 2*Pi.  Defaults to 2*Pi.
- `organize_cloud` (bool) - Whether to organize the cloud by ring (True), or to use the order as it comes directly from the driver (False).  Defaults to True.
- `target_frame` (string) - The coordinate frame to apply to the generated point cloud header before publishing.  If the empty string (the default), the frame is passed along from the driver packet.  If this frame is different than the `fixed_frame`, a transformation to this coordinate frame is performed while creating the pointcloud.
- `fixed_frame` (string) - The fixed coordinate frame to transform the data from.