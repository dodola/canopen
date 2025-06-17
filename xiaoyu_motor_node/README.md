# Xiaoyu Motor ROS2 Node (`xiaoyu_motor_node`)

This ROS2 package provides a node to interface with Xiaoyu motor devices using CANopen.

## Overview

The `motor_node` initializes a CANopen network, communicates with a specified CANopen node (assumed to be a CiA 402 compliant motor drive), and exposes ROS2 services and topics for control and monitoring.

**Note:** This node requires a running CAN interface and a connected/configured CANopen motor device for full functionality. The EDS file for the motor (e.g., `eds/cia402_slave.eds`) also needs to be accessible at the path specified in the node.

## Dependencies

*   ROS 2 (Humble or a compatible version for your OS)
*   `python-canopen` library (`pip install python-canopen`)
*   The CAN interface driver for your hardware (e.g., Kvaser, SocketCAN)

## Building the Package

1.  Ensure you have a ROS 2 development environment sourced.
2.  Place this package in your ROS 2 workspace's `src/` directory.
3.  Navigate to the root of your workspace.
4.  Build the package:
    ```bash
    colcon build --packages-select xiaoyu_motor_node
    ```

## Running the Node

1.  Source your workspace's setup files:
    ```bash
    source install/setup.bash
    ```
2.  Launch the node:
    ```bash
    ros2 run xiaoyu_motor_node motor_node
    ```
    The node will attempt to connect to the CAN interface and initialize the motor node as configured within `motor_node.py` (e.g., interface 'kvaser', channel 0, bitrate 1000000, node ID 35). These may need adjustment.

## Services

The node provides the following services under the `/xiaoyu/motor_device/` namespace:

*   **`init`** (`std_srvs/srv/Trigger`): Placeholder for full motor initialization sequence.
*   **`motor_soft_reset`** (`std_srvs/srv/Trigger`): Attempts a fault reset on the motor.
*   **`nmt_reset_node`** (`std_srvs/srv/Trigger`): Sends NMT reset communication command to the configured node.
*   **`nmt_start_node`** (`std_srvs/srv/Trigger`): Sends NMT start node command to the configured node.
*   **`position_mode`** (`std_srvs/srv/Trigger`): Placeholder to switch motor to position control mode.
*   **`recover`** (`std_srvs/srv/Trigger`): Placeholder for a motor recovery sequence.
*   **`sdo_read`** (`xiaoyu_motor_node/srv/CORead`): Reads an SDO object from the configured node.
    *   Request: `uint16 index`, `uint8 subindex`
    *   Response: `bool success`, `uint32 data`
*   **`sdo_write`** (`xiaoyu_motor_node/srv/COWrite`): Writes to an SDO object on the configured node.
    *   Request: `uint16 index`, `uint8 subindex`, `uint32 data`
    *   Response: `bool success`
*   **`target`** (`xiaoyu_motor_node/srv/COTargetDouble`): Sends a target value (e.g., velocity, position) to the motor.
    *   Request: `float64 target`
    *   Response: `bool success`
*   **`update_firmware`** (`xiaoyu_motor_node/srv/FirmwareUpdate`): Placeholder for firmware update functionality.
    *   Request: `string firmware_url`, `uint32 target_addr`, `uint32 packet_size`, `float32 frame_delay`, `bool debug`
    *   Response: `bool success`, `string message`
*   **`velocity_mode`** (`std_srvs/srv/Trigger`): Placeholder to switch motor to velocity control mode.
*   **`sdo_read_id`** (`xiaoyu_motor_node/srv/COReadID`): Reads SDO from a specific node ID. (Conceptual with current single-node setup)
    *   Request: `uint8 nodeid`, `uint16 index`, `uint8 subindex`, `uint8 canopen_datatype`
    *   Response: `bool success`, `uint32 data`
*   **`sdo_write_id`** (`xiaoyu_motor_node/srv/COWriteID`): Writes SDO to a specific node ID. (Conceptual)
    *   Request: `uint8 nodeid`, `uint16 index`, `uint8 subindex`, `uint32 data`, `uint8 canopen_datatype`
    *   Response: `bool success`
*   **`set_node_id`** (`xiaoyu_motor_node/srv/CONode`): Placeholder to target a different node ID. (Conceptual)
    *   Request: `uint8 nodeid`
    *   Response: `bool success`
*   **`nmt_command_id`** (`xiaoyu_motor_node/srv/CONmtID`): Sends NMT command to a specific node ID. (Conceptual)
    *   Request: `uint8 nmtcommand`, `uint8 nodeid`
    *   Response: `bool success`
*   **`set_heartbeat_id`** (`xiaoyu_motor_node/srv/COHeartbeatID`): Configures heartbeat for a specific node ID. (Conceptual)
    *   Request: `uint8 nodeid`, `uint16 heartbeat`
    *   Response: `bool success`

## Topics

*   **`/xiaoyu/motor_device/joint_states`** (`sensor_msgs/msg/JointState`): Publishes motor position, velocity (if available from PDOs/SDOs).
*   **`/xiaoyu/motor_device/nmt_state`** (`std_msgs/msg/String`): Publishes the NMT state of the configured CANopen node.

## Custom Messages and Services

This package defines its own services and messages:
*   `srv/COWriteID.srv`
*   `srv/COWrite.srv`
*   `srv/COTargetDouble.srv`
*   `srv/COReadID.srv`
*   `srv/CORead.srv`
*   `srv/CONode.srv`
*   `srv/CONmtID.srv`
*   `srv/COHeartbeatID.srv`
*   `srv/FirmwareUpdate.srv`
*   `msg/COData.msg`

These are generated during the build process.
