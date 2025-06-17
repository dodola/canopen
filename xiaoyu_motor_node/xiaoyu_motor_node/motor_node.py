import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from canopen_interfaces.srv import CORead, COWrite, COTargetDouble
from xiaoyu_motor_can.srv import FirmwareUpdate # Assuming this is available
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import canopen
import time # For delays in CANopen init

# Define placeholder for EDS file path, should be configurable
EDS_FILE_PATH = 'eds/cia402_slave.eds'
# Define placeholder for CANopen node ID
CANOPEN_NODE_ID = 35
# Define placeholder for target SDO (example: target velocity 0x60FF)
TARGET_VELOCITY_SDO_INDEX = 0x60FF
TARGET_VELOCITY_SDO_SUBINDEX = 0
# Define placeholder for target position SDO (example: target position 0x607A)
TARGET_POSITION_SDO_INDEX = 0x607A
TARGET_POSITION_SDO_SUBINDEX = 0


class MotorNode(Node):
    def __init__(self):
        super().__init__('xiaoyu_motor_device')
        self.get_logger().info('Motor node started')
        self.network = None
        self.node = None

        try:
            # Initialize CANopen network
            self.network = canopen.Network()
            # Try to connect to the CAN bus
            # Placeholder values - replace with actual configuration
            self.network.connect(interface='kvaser', channel='0', bitrate=1000000)
            # self.network.connect(bustype='socketcan', channel='can0', bitrate=1000000)
            self.get_logger().info('Successfully connected to CAN bus.')

            # Add CANopen node
            # Ensure EDS_FILE_PATH is correct and accessible
            self.node = canopen.BaseNode402(CANOPEN_NODE_ID, EDS_FILE_PATH)
            self.network.add_node(self.node)
            self.get_logger().info(f'Added CANopen node ID {CANOPEN_NODE_ID} with EDS {EDS_FILE_PATH}.')

            # Basic NMT state changes (example from simple_ds402_node.py)
            self.get_logger().info('Resetting node communication...')
            self.node.nmt.state = 'RESET COMMUNICATION'
            self.node.nmt.wait_for_bootup(15) # Increased timeout
            self.get_logger().info(f'Node {CANOPEN_NODE_ID} booted up.')

            self.get_logger().info(f'Reading NMT state after reset: {self.node.nmt.state}')

            # Example: Configure PDOs (specific to your device EDS)
            # self.node.tpdo.read()
            # self.node.rpdo.read()
            # self.node.tpdo[1].enabled = True
            # self.node.tpdo[1].trans_type = 1 # Event-driven
            # self.node.tpdo[1].event_timer = 100 # ms
            # self.node.tpdo.save()

            self.get_logger().info('Setting node to OPERATIONAL...')
            self.node.nmt.state = 'OPERATIONAL'
            # Wait for operational state (optional, good for debugging)
            # time.sleep(1)
            self.get_logger().info(f'NMT state after attempting to set OPERATIONAL: {self.node.nmt.state}')


        except Exception as e:
            self.get_logger().error(f'Failed to initialize CANopen network or node: {e}')
            self.node = None # Ensure node is None if setup fails
            # Consider how to handle node failure, e.g., prevent service/topic creation or enter a degraded mode


        # Create publishers
        self.joint_state_publisher = self.create_publisher(JointState, '/xiaoyu/motor_device/joint_states', 10)
        self.nmt_state_publisher = self.create_publisher(String, '/xiaoyu/motor_device/nmt_state', 10)

        # Create timers for publishing
        self.joint_state_timer = self.create_timer(1.0, self.publish_joint_states) # Publish every 1 second
        self.nmt_state_timer = self.create_timer(1.0, self.publish_nmt_state) # Publish every 1 second

        # Create service servers
        self.create_service(Trigger, '/xiaoyu/motor_device/init', self.init_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/motor_soft_reset', self.motor_soft_reset_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/nmt_reset_node', self.nmt_reset_node_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/nmt_start_node', self.nmt_start_node_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/position_mode', self.position_mode_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/recover', self.recover_callback)
        self.create_service(CORead, '/xiaoyu/motor_device/sdo_read', self.sdo_read_callback)
        self.create_service(COWrite, '/xiaoyu/motor_device/sdo_write', self.sdo_write_callback)
        self.create_service(COTargetDouble, '/xiaoyu/motor_device/target', self.target_callback)
        self.create_service(FirmwareUpdate, '/xiaoyu/motor_device/update_firmware', self.update_firmware_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/velocity_mode', self.velocity_mode_callback)

    # Service callbacks
    def init_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/init called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            # Placeholder for full initialization sequence
            # This would involve:
            # 1. NMT reset node (if not already done or if re-init is needed)
            #    self.node.nmt.state = 'RESET COMMUNICATION'
            #    self.node.nmt.wait_for_bootup(10)
            # 2. Configure PDOs (if not done or needs reconfiguration)
            #    self.node.tpdo.read()
            #    self.node.rpdo.read()
            #    # Example: self.node.tpdo[1].enabled = True ...
            #    self.node.tpdo.save()
            #    self.node.rpdo.save()
            # 3. Set up state machine (fault reset, enable operation)
            #    # Fault reset (example: 0x6040, controlword)
            #    self.node.sdo[0x6040].raw = 0x80 # Fault reset
            #    time.sleep(0.1)
            #    self.node.sdo[0x6040].raw = 0x06 # Shutdown
            #    time.sleep(0.1)
            #    self.node.sdo[0x6040].raw = 0x0F # Switch on and enable operation
            # 4. Set to OPERATIONAL
            #    self.node.nmt.state = 'OPERATIONAL'
            self.get_logger().info("CANopen node init placeholder: NMT reset, PDO setup, state machine setup.")
            response.success = True
            response.message = 'CANopen init sequence placeholder executed.'
        except Exception as e:
            self.get_logger().error(f"Error in init_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def motor_soft_reset_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/motor_soft_reset called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            # Example for fault reset using Controlword (0x6040)
            # This sequence depends on the motor drive's state machine (CiA 402)
            controlword_sdo = self.node.sdo[0x6040] # Controlword SDO
            self.get_logger().info(f"Current Controlword: {controlword_sdo.raw:#06x}")
            # Fault reset: transition 7 (from Fault state)
            controlword_sdo.raw = 0x80  # Bit 7 = 1 (Fault reset)
            time.sleep(0.1) # Give some time for the device to react
            self.get_logger().info(f"Controlword after 0x80: {controlword_sdo.raw:#06x}")
            # After fault reset, device might go to Switch On Disabled.
            # To enable, typically: Shutdown (0x06) -> Switch On (0x07) -> Enable Operation (0x0F)
            # For simplicity here, just showing the fault reset command.
            # A more complete sequence might be:
            # controlword_sdo.raw = 0x06 # -> Switch On Disabled
            # time.sleep(0.1)
            # controlword_sdo.raw = 0x07 # -> Switched On
            # time.sleep(0.1)
            # controlword_sdo.raw = 0x0F # -> Operation Enabled (if no fault)

            # Alternative using RPDO if Controlword is mapped to an RPDO
            # self.node.rpdo[1]['Controlword'].raw = 0x80 # Example: RPDO 1, Controlword entry
            # time.sleep(0.1)
            # self.node.rpdo[1]['Controlword'].raw = 0x06 # Then to shutdown
            # ... and so on to enable operation

            self.get_logger().info("Attempted motor fault reset (soft reset).")
            response.success = True
            response.message = 'Motor soft reset command sent.'
        except Exception as e:
            self.get_logger().error(f"Error in motor_soft_reset_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def nmt_reset_node_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/nmt_reset_node called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            self.node.nmt.state = 'RESET COMMUNICATION' # Or 'RESET' depending on desired reset type
            self.get_logger().info(f"NMT state set to RESET COMMUNICATION. Waiting for boot-up...")
            self.node.nmt.wait_for_bootup(10) # Wait for the node to boot up
            self.get_logger().info(f"Node {CANOPEN_NODE_ID} booted up. Current NMT state: {self.node.nmt.state}")
            response.success = True
            response.message = 'NMT reset node command sent.'
        except Exception as e:
            self.get_logger().error(f"Error in nmt_reset_node_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def nmt_start_node_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/nmt_start_node called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            # Sequence to get to OPERATIONAL, assuming it's currently PRE-OPERATIONAL
            # Fault reset first (if needed, check statusword 0x6041)
            # statusword = self.node.sdo[0x6041].raw
            # if (statusword & 0x0F) == 0x08: # Check if in Fault state
            #    self.node.sdo[0x6040].raw = 0x80 # Fault reset
            #    time.sleep(0.1)

            # Transition from Switch On Disabled to Operation Enabled
            # self.node.sdo[0x6040].raw = 0x06 # Shutdown -> moves to Switch on disabled
            # time.sleep(0.1)
            # self.node.sdo[0x6040].raw = 0x07 # Switch on -> moves to Switched on
            # time.sleep(0.1)
            # self.node.sdo[0x6040].raw = 0x0F # Enable operation -> moves to Operation Enabled
            # time.sleep(0.1)

            self.node.nmt.state = 'OPERATIONAL'
            self.get_logger().info(f"NMT state set to OPERATIONAL. Current NMT state: {self.node.nmt.state}")
            response.success = True
            response.message = 'NMT start node command sent.'
        except Exception as e:
            self.get_logger().error(f"Error in nmt_start_node_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def position_mode_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/position_mode called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            # Placeholder for setting position mode (e.g., writing to SDO 0x6060, Modes of Operation)
            # Example: self.node.sdo[0x6060].raw = 1 # Profile Position Mode (PP)
            # Ensure motor is in a state that allows mode change (e.g., Switched On)
            self.get_logger().info("Placeholder: Set motor to position mode via SDO 0x6060.")
            response.success = True
            response.message = 'Position mode setting placeholder executed.'
        except Exception as e:
            self.get_logger().error(f"Error in position_mode_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def recover_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/recover called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            # Placeholder for a recovery sequence
            # This might involve fault reset, NMT reset, re-initialization etc.
            self.get_logger().info("Placeholder: Attempting recovery sequence (e.g., fault reset, NMT reset).")
            # Example: self.motor_soft_reset_callback(None, Trigger.Response())
            # Example: self.nmt_reset_node_callback(None, Trigger.Response())
            # Example: self.init_callback(None, Trigger.Response())
            response.success = True
            response.message = 'Recovery sequence placeholder executed.'
        except Exception as e:
            self.get_logger().error(f"Error in recover_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def sdo_read_callback(self, request, response):
        self.get_logger().info(f'Service /xiaoyu/motor_device/sdo_read called with index: {request.index:#06x}, subindex: {request.subindex:#04x}')
        if not self.node:
            response.success = False
            # response.data will be default (0)
            self.get_logger().error("CANopen node not initialized.")
            return response
        try:
            # Ensure index and subindex are integers if they are not already
            idx = int(request.index)
            subidx = int(request.subindex)

            # data = self.node.sdo[idx][subidx].raw # Access SDO by int index/subindex
            # For some EDS files, string access might be needed if names are defined:
            # data = self.node.sdo['Manufacturer device name'].raw # Example string access
            # Check if the SDO object exists before trying to read
            if idx in self.node.sdo:
                if subidx in self.node.sdo[idx]:
                    data = self.node.sdo[idx][subidx].raw
                    response.data = data
                    response.success = True
                    self.get_logger().info(f"SDO Read [{idx:#06x}][{subidx:#04x}]: Value = {data} ({data:#x})")
                else:
                    response.success = False
                    self.get_logger().error(f"SDO Read Error: Subindex {subidx:#04x} not found for index {idx:#06x}.")
            else:
                response.success = False
                self.get_logger().error(f"SDO Read Error: Index {idx:#06x} not found.")

        except Exception as e:
            self.get_logger().error(f"Error in sdo_read_callback: {e}")
            response.success = False
            # response.data will be default (0)
        return response

    def sdo_write_callback(self, request, response):
        self.get_logger().info(f'Service /xiaoyu/motor_device/sdo_write called with index: {request.index:#06x}, subindex: {request.subindex:#04x}, data: {request.data} ({request.data:#x})')
        if not self.node:
            response.success = False
            self.get_logger().error("CANopen node not initialized.")
            return response
        try:
            idx = int(request.index)
            subidx = int(request.subindex)
            data_to_write = int(request.data)

            # self.node.sdo[idx][subidx].raw = data_to_write
            if idx in self.node.sdo:
                if subidx in self.node.sdo[idx]:
                    self.node.sdo[idx][subidx].raw = data_to_write
                    response.success = True
                    self.get_logger().info(f"SDO Write [{idx:#06x}][{subidx:#04x}]: Value = {data_to_write} ({data_to_write:#x})")
                else:
                    response.success = False
                    self.get_logger().error(f"SDO Write Error: Subindex {subidx:#04x} not found for index {idx:#06x}.")
            else:
                response.success = False
                self.get_logger().error(f"SDO Write Error: Index {idx:#06x} not found.")

        except Exception as e:
            self.get_logger().error(f"Error in sdo_write_callback: {e}")
            response.success = False
        return response

    def target_callback(self, request, response):
        # Assuming this sets a target value like velocity or position
        self.get_logger().info(f'Service /xiaoyu/motor_device/target called with data: {request.data}')
        if not self.node:
            response.success = False
            self.get_logger().error("CANopen node not initialized.")
            return response
        try:
            # Placeholder: Determine if this is for velocity or position based on current mode
            # current_mode = self.node.sdo[0x6061].raw # Modes of Operation Display
            # if current_mode == 3: # Profile Velocity Mode
            #    self.node.sdo[TARGET_VELOCITY_SDO_INDEX][TARGET_VELOCITY_SDO_SUBINDEX].raw = int(request.data)
            #    self.get_logger().info(f"Set target velocity to {request.data} via SDO {TARGET_VELOCITY_SDO_INDEX:#06x}")
            # elif current_mode == 1: # Profile Position Mode
            #    self.node.sdo[TARGET_POSITION_SDO_INDEX][TARGET_POSITION_SDO_SUBINDEX].raw = int(request.data)
            #    # May need to trigger movement with control word
            #    self.get_logger().info(f"Set target position to {request.data} via SDO {TARGET_POSITION_SDO_INDEX:#06x}")
            # else:
            #    self.get_logger().warning(f"Target service called but not in a supported mode (current_mode={current_mode})")
            #    response.success = False
            #    return response
            self.get_logger().info("Placeholder: Set target via SDO (e.g., 0x60FF for velocity, 0x607A for position).")
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error in target_callback: {e}")
            response.success = False
        return response

    def update_firmware_callback(self, request, response):
        self.get_logger().info(f'Service /xiaoyu/motor_device/update_firmware called with path: {request.path}, node_id: {request.node_id}')
        # Actual firmware update via CANopen is complex and device-specific.
        # It usually involves a specific SDO protocol or NMT commands for bootloader mode.
        # This is a placeholder.
        self.get_logger().warning("Firmware update via CANopen is not implemented. This is a placeholder.")
        response.success = False # Mark as not successful as it's not implemented
        response.message = 'Firmware update not implemented.'
        return response

    def velocity_mode_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/velocity_mode called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            # Placeholder for setting velocity mode (e.g., writing to SDO 0x6060, Modes of Operation)
            # Example: self.node.sdo[0x6060].raw = 3 # Profile Velocity Mode (PV)
            # Ensure motor is in a state that allows mode change
            self.get_logger().info("Placeholder: Set motor to velocity mode via SDO 0x6060.")
            response.success = True
            response.message = 'Velocity mode setting placeholder executed.'
        except Exception as e:
            self.get_logger().error(f"Error in velocity_mode_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    # Timer callbacks for publishing
    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['motor_joint']

        if self.node and self.node.nmt.state == 'OPERATIONAL':
            try:
                # Example: Read actual position from TPDO or SDO (0x6064: Position actual value)
                # actual_position_raw = self.node.sdo[0x6064].raw
                # actual_position_phys = self.node.sdo[0x6064].phys # If scaling defined in EDS

                # Example: Read actual velocity from TPDO or SDO (0x606C: Velocity actual value)
                # actual_velocity_raw = self.node.sdo[0x606C].raw
                # actual_velocity_phys = self.node.sdo[0x606C].phys

                # Placeholder values if SDOs are not readable or for testing
                actual_position_phys = 0.0
                actual_velocity_phys = 0.0

                # Check if TPDOs are configured and use them if available
                # if 1 in self.node.tpdo and 'Position actual value' in self.node.tpdo[1]:
                #    actual_position_phys = self.node.tpdo[1]['Position actual value'].phys
                # if 1 in self.node.tpdo and 'Velocity actual value' in self.node.tpdo[1]:
                #    actual_velocity_phys = self.node.tpdo[1]['Velocity actual value'].phys

                msg.position = [actual_position_phys]
                msg.velocity = [actual_velocity_phys]
                msg.effort = [0.0] # Effort usually requires specific SDO/TPDO
                # self.get_logger().info(f"Publishing JointState: Pos={actual_position_phys}, Vel={actual_velocity_phys}")
            except Exception as e:
                # self.get_logger().error(f"Error reading CANopen data for JointState: {e}")
                msg.position = [0.0]
                msg.velocity = [0.0]
                msg.effort = [0.0]
        else:
            msg.position = [0.0] # Default if node not ready
            msg.velocity = [0.0]
            msg.effort = [0.0]
            if not self.node:
                pass # self.get_logger().warn("CANopen node not available for JointState publishing.", throttle_duration_sec=5)
            elif self.node.nmt.state != 'OPERATIONAL':
                pass # self.get_logger().warn(f"CANopen node not OPERATIONAL (state: {self.node.nmt.state}) for JointState publishing.", throttle_duration_sec=5)

        self.joint_state_publisher.publish(msg)

    def publish_nmt_state(self):
        msg = String()
        if self.node:
            try:
                msg.data = str(self.node.nmt.state)
            except Exception as e:
                # self.get_logger().error(f"Error reading NMT state: {e}")
                msg.data = "ERROR_READING_NMT"
        else:
            msg.data = "NOT_INITIALIZED"
        # self.get_logger().info(f"Publishing NMT State: {msg.data}")
        self.nmt_state_publisher.publish(msg)

    def destroy_node_custom(self):
        self.get_logger().info("Shutting down CANopen node...")
        if self.node:
            try:
                # Set node to PRE-OPERATIONAL before disconnecting
                self.get_logger().info("Setting node to PRE-OPERATIONAL...")
                self.node.nmt.state = 'PRE-OPERATIONAL'
                # Add delay or check if state change was successful if necessary
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f"Error setting node to PRE-OPERATIONAL: {e}")

            # Stop node guarding or heartbeat if used (not explicitly started in this example)
            # try:
            #     self.node.nmt.stop_node_guarding()
            #     self.get_logger().info("Stopped node guarding.")
            # except Exception as e:
            #     self.get_logger().error(f"Error stopping node guarding: {e}")

            # Stop SYNC producer if this node was a SYNC producer (not in this example)
            # try:
            #    if self.network.is_sync_producer:
            #        self.network.sync.stop()
            #        self.get_logger().info("Stopped SYNC producer.")
            # except Exception as e:
            #    self.get_logger().error(f"Error stopping SYNC producer: {e}")

        if self.network:
            try:
                self.get_logger().info("Disconnecting from CAN bus...")
                self.network.disconnect()
                self.get_logger().info("Successfully disconnected from CAN bus.")
            except Exception as e:
                self.get_logger().error(f"Error disconnecting from CAN bus: {e}")

        super().destroy_node() # Call the parent class's destroy_node


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().fatal(f"Unhandled exception in main spin: {e}")
    finally:
        node.get_logger().info("Initiating shutdown...")
        node.destroy_node_custom() # Use custom destroy method
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()
