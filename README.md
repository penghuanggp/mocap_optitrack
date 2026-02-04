# mocap_optitrack

ROS 2 driver for OptiTrack motion capture systems using the NatNet protocol.

## Features

- **Rigid Body Tracking**: Publishes pose, odometry, and TF for configured rigid bodies
- **Single Marker Tracking**: Publishes individual marker positions as `PointStamped` messages

## Configuration

Create a YAML configuration file (see `config/mocap.yaml`):

```yaml
mocap_node:
    ros__parameters:
        # Rigid body configuration
        rigid_bodies:
            1:
                pose: "robot1/pose"
                odom: "robot1/odom"
                tf: tf
                child_frame_id: "robot1/base_link"
                parent_frame_id: "world"
        
        # OptiTrack server settings
        optitrack_config:
            multicast_address: "239.255.42.99"
            command_port: 1510
            data_port: 1511
            enable_optitrack: true
            version: [4, 0, 0, 0]
            
            # Marker publishing (optional)
            enable_markers: true
            markers_topic: "markers"
```

## Marker Publishing

When `enable_markers` is set to `true`, individual markers are published as `geometry_msgs/msg/PointStamped` messages.

### Topics

- `<markers_topic>/marker_<id>`: Position of marker with given ID
  - Labeled markers: Use their OptiTrack ID (e.g., `markers/marker_1`)
  - Unlabeled markers: Use IDs starting from 10000 (e.g., `markers/marker_10000`)

### Behavior

- Publishers are created dynamically when new markers are detected
- Stale publishers are automatically removed when markers disappear
- Both labeled and unlabeled markers are tracked
- Unlabeled markers use IDs starting from 10000 to avoid collision with labeled marker IDs
- Frame ID is `optitrack`

### Example: Subscribing to a Marker

```bash
ros2 topic echo /markers/marker_1
```

## Launch

```bash
ros2 launch mocap_optitrack mocap.launch.py
```

Or with a custom config:

```bash
ros2 launch mocap_optitrack mocap.launch.py mocap_config:=/path/to/your/config.yaml
```

## License

BSD-3-Clause - See source files for full license text.
