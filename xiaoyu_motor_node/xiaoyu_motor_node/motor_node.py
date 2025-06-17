import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
# Updated imports for locally defined services
from xiaoyu_motor_node.srv import CORead
from xiaoyu_motor_node.srv import COWrite
from xiaoyu_motor_node.srv import COTargetDouble
from xiaoyu_motor_node.srv import FirmwareUpdate
from xiaoyu_motor_node.srv import COReadID, COWriteID, CONode, CONmtID, COHeartbeatID
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
        super().__init__('xiaoyu_motor_device') # Changed from 'motor_node' to 'xiaoyu_motor_device' earlier
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

        # Create service servers (using updated local service types)
        self.create_service(Trigger, '/xiaoyu/motor_device/init', self.init_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/motor_soft_reset', self.motor_soft_reset_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/nmt_reset_node', self.nmt_reset_node_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/nmt_start_node', self.nmt_start_node_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/position_mode', self.position_mode_callback)
        self.create_service(Trigger, '/xiaoyu/motor_device/recover', self.recover_callback)
        self.create_service(CORead, '/xiaoyu/motor_device/sdo_read', self.sdo_read_callback) # Uses local CORead
        self.create_service(COWrite, '/xiaoyu/motor_device/sdo_write', self.sdo_write_callback) # Uses local COWrite
        self.create_service(COTargetDouble, '/xiaoyu/motor_device/target', self.target_callback) # Uses local COTargetDouble
        self.create_service(FirmwareUpdate, '/xiaoyu/motor_device/update_firmware', self.update_firmware_callback) # Uses local FirmwareUpdate
        self.create_service(Trigger, '/xiaoyu/motor_device/velocity_mode', self.velocity_mode_callback)

        # New services
        self.co_read_id_service = self.create_service(COReadID, '/xiaoyu/motor_device/sdo_read_id', self.co_read_id_callback)
        self.co_write_id_service = self.create_service(COWriteID, '/xiaoyu/motor_device/sdo_write_id', self.co_write_id_callback)
        self.co_node_service = self.create_service(CONode, '/xiaoyu/motor_device/set_node_id', self.co_node_callback)
        self.co_nmt_id_service = self.create_service(CONmtID, '/xiaoyu/motor_device/nmt_command_id', self.co_nmt_id_callback)
        self.co_heartbeat_id_service = self.create_service(COHeartbeatID, '/xiaoyu/motor_device/set_heartbeat_id', self.co_heartbeat_id_callback)

    # Service callbacks (definitions remain the same, only types used in creation are changed)
    def init_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/init called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
            # Placeholder for full initialization sequence
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
            controlword_sdo = self.node.sdo[0x6040]
            self.get_logger().info(f"Current Controlword: {controlword_sdo.raw:#06x}")
            controlword_sdo.raw = 0x80
            time.sleep(0.1)
            self.get_logger().info(f"Controlword after 0x80: {controlword_sdo.raw:#06x}")
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
            self.node.nmt.state = 'RESET COMMUNICATION'
            self.get_logger().info(f"NMT state set to RESET COMMUNICATION. Waiting for boot-up...")
            self.node.nmt.wait_for_bootup(10)
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
            self.get_logger().info("Placeholder: Attempting recovery sequence (e.g., fault reset, NMT reset).")
            response.success = True
            response.message = 'Recovery sequence placeholder executed.'
        except Exception as e:
            self.get_logger().error(f"Error in recover_callback: {e}")
            response.success = False
            response.message = f"Error: {e}"
        return response

    def sdo_read_callback(self, request, response): # request is CORead.Request, response is CORead.Response
        self.get_logger().info(f'Service /xiaoyu/motor_device/sdo_read called with index: {request.index:#06x}, subindex: {request.subindex:#04x}')
        if not self.node:
            response.success = False
            self.get_logger().error("CANopen node not initialized.")
            return response
        try:
            idx = int(request.index)
            subidx = int(request.subindex)
            if idx in self.node.sdo:
                if subidx in self.node.sdo[idx]:
                    data = self.node.sdo[idx][subidx].raw
                    response.data = data # Assuming CORead.Response has a 'data' field
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
        return response

    def sdo_write_callback(self, request, response): # request is COWrite.Request, response is COWrite.Response
        self.get_logger().info(f'Service /xiaoyu/motor_device/sdo_write called with index: {request.index:#06x}, subindex: {request.subindex:#04x}, data: {request.data} ({request.data:#x})')
        if not self.node:
            response.success = False
            self.get_logger().error("CANopen node not initialized.")
            return response
        try:
            idx = int(request.index)
            subidx = int(request.subindex)
            data_to_write = int(request.data) # Assuming COWrite.Request has a 'data' field
            if idx in self.node.sdo:
                if subidx in self.node.sdo[idx]:
                    self.node.sdo[idx][subidx].raw = data_to_write
                    response.success = True # Assuming COWrite.Response has a 'success' field
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

    def target_callback(self, request, response): # request is COTargetDouble.Request, response is COTargetDouble.Response
        self.get_logger().info(f'Service /xiaoyu/motor_device/target called with data: {request.target}') # Assuming COTargetDouble.Request has 'target'
        if not self.node:
            response.success = False
            self.get_logger().error("CANopen node not initialized.")
            return response
        try:
            self.get_logger().info("Placeholder: Set target via SDO (e.g., 0x60FF for velocity, 0x607A for position).")
            # Example: self.node.sdo[TARGET_VELOCITY_SDO_INDEX][TARGET_VELOCITY_SDO_SUBINDEX].raw = int(request.target)
            response.success = True # Assuming COTargetDouble.Response has 'success'
        except Exception as e:
            self.get_logger().error(f"Error in target_callback: {e}")
            response.success = False
        return response

    def update_firmware_callback(self, request, response): # request is FirmwareUpdate.Request, response is FirmwareUpdate.Response
        self.get_logger().info(f'Service /xiaoyu/motor_device/update_firmware called with path: {request.firmware_url}, node_id (not used by this node directly for CANopen node ID): {request.target_addr}') # Field names from FirmwareUpdate.srv
        self.get_logger().warning("Firmware update via CANopen is not implemented. This is a placeholder.")
        response.success = False
        response.message = 'Firmware update not implemented.' # Assuming FirmwareUpdate.Response has 'message'
        return response

    def velocity_mode_callback(self, request, response):
        self.get_logger().info('Service /xiaoyu/motor_device/velocity_mode called')
        if not self.node:
            response.success = False
            response.message = "CANopen node not initialized."
            return response
        try:
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
                actual_position_phys = 0.0
                actual_velocity_phys = 0.0
                # Placeholder: Read actual_position_phys and actual_velocity_phys from self.node.sdo or self.node.tpdo
                msg.position = [actual_position_phys]
                msg.velocity = [actual_velocity_phys]
                msg.effort = [0.0]
            except Exception as e:
                msg.position = [0.0]
                msg.velocity = [0.0]
                msg.effort = [0.0]
        else:
            msg.position = [0.0]
            msg.velocity = [0.0]
            msg.effort = [0.0]
        self.joint_state_publisher.publish(msg)

    def publish_nmt_state(self):
        msg = String()
        if self.node:
            try:
                msg.data = str(self.node.nmt.state)
            except Exception as e:
                msg.data = "ERROR_READING_NMT"
        else:
            msg.data = "NOT_INITIALIZED"
        self.nmt_state_publisher.publish(msg)

    # Callbacks for new services
    def co_read_id_callback(self, request, response):
        self.get_logger().info(
            f'Service /xiaoyu/motor_device/sdo_read_id called for node {request.nodeid}, index {request.index}, subindex {request.subindex}, type {request.canopen_datatype}')
        # Placeholder CANopen logic for reading SDO from specific node
        if self.node and self.network: # Basic check
            try:
                # This is a conceptual example; python-canopen's BaseNode402 is associated with one node.
                # Accessing other nodes would require a different structure or network[node_id].sdo
                self.get_logger().warn("SDO Read ID: Accessing specific node IDs not directly supported by self.node. Conceptual.")
                # Example: val = self.network[request.nodeid].sdo[request.index][request.subindex].raw
                # response.data = val
                response.success = False # Mark as not truly implemented yet
                response.data = 0
            except Exception as e:
                self.get_logger().error(f'Failed to read SDO ID: {e}')
                response.success = False
                response.data = 0
        else:
            self.get_logger().warn('CANopen network/node not initialized.')
            response.success = False
            response.data = 0
        return response

    def co_write_id_callback(self, request, response):
        self.get_logger().info(
            f'Service /xiaoyu/motor_device/sdo_write_id called for node {request.nodeid}, index {request.index}, subindex {request.subindex}, data {request.data}, type {request.canopen_datatype}')
        if self.node and self.network:
            try:
                self.get_logger().warn("SDO Write ID: Accessing specific node IDs not directly supported by self.node. Conceptual.")
                # Example: self.network[request.nodeid].sdo[request.index][request.subindex].raw = request.data
                response.success = False # Mark as not truly implemented yet
            except Exception as e:
                self.get_logger().error(f'Failed to write SDO ID: {e}')
                response.success = False
        else:
            self.get_logger().warn('CANopen network/node not initialized.')
            response.success = False
        # response.success = True # Placeholder, corrected to be conditional
        return response

    def co_node_callback(self, request, response):
        self.get_logger().info(f'Service /xiaoyu/motor_device/set_node_id called for node {request.nodeid}')
        # Placeholder: Logic to switch active node or store target node ID
        self.get_logger().warn("Set Node ID: Conceptual. Current structure uses one self.node.")
        response.success = True # Placeholder
        return response

    def co_nmt_id_callback(self, request, response):
        self.get_logger().info(
            f'Service /xiaoyu/motor_device/nmt_command_id called for node {request.nodeid} with command {request.nmtcommand}')
        # NMT commands from canopen.nmt.NMT_COMMANDS, e.g., 'RESET COMMUNICATION', 'START NODE'
        # Example: NMT_COMMAND_MAP = {1: 'START NODE', 2: 'STOP NODE', 128: 'RESET NODE', 129: 'RESET COMMUNICATION'}
        # command_str = NMT_COMMAND_MAP.get(request.nmtcommand, "UNKNOWN COMMAND")
        if self.network:
            try:
                # This assumes nmtcommand is a string like 'RESET COMMUNICATION' or an int that needs mapping
                # self.network[request.nodeid].nmt.state = command_str
                self.get_logger().warn("NMT Command ID: Conceptual for specific node ID if not self.node.")
                response.success = False # Mark as not truly implemented yet
            except Exception as e:
                self.get_logger().error(f'Failed to send NMT command ID: {e}')
                response.success = False
        else:
            self.get_logger().warn('CANopen network not initialized.')
            response.success = False
        return response

    def co_heartbeat_id_callback(self, request, response):
        self.get_logger().info(
            f'Service /xiaoyu/motor_device/set_heartbeat_id called for node {request.nodeid} with interval {request.heartbeat} ms')
        if self.network:
            try:
                # Example: self.network[request.nodeid].sdo[0x1017].raw = request.heartbeat
                self.get_logger().warn("Set Heartbeat ID: Conceptual for specific node ID.")
                response.success = False # Mark as not truly implemented yet
            except Exception as e:
                self.get_logger().error(f'Failed to set heartbeat ID: {e}')
                response.success = False
        else:
            self.get_logger().warn('CANopen network not initialized.')
            response.success = False
        return response

    def destroy_node_custom(self):
        self.get_logger().info("Shutting down CANopen node...")
        if self.node:
            try:
                self.get_logger().info("Setting node to PRE-OPERATIONAL...")
                self.node.nmt.state = 'PRE-OPERATIONAL'
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f"Error setting node to PRE-OPERATIONAL: {e}")
        if self.network:
            try:
                self.get_logger().info("Disconnecting from CAN bus...")
                self.network.disconnect()
                self.get_logger().info("Successfully disconnected from CAN bus.")
            except Exception as e:
                self.get_logger().error(f"Error disconnecting from CAN bus: {e}")
        super().destroy_node()


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
        node.destroy_node_custom()
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()
