from dynamixel_sdk import * # Uses DYNAMIXEL SDK library
import time

import logging
from physical_robot.utils import register_logger

logger = register_logger(logger_name=__name__, log_filename='dxl_controller', level=logging.INFO, std_err=False)


ADDR_OPERATING_MODE = 11

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_VELOCITY = 104
ADDR_PROF_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132


TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


# Operating Modes
VELOCITY_CONTROL_MODE = 1
POSITION_CONTROL_MODE = 3
EXTENDED_POSITION_CONTROL_MODE = 4
PWM_CONTROL_MODE = 16

# Protocol version
PROTOCOL_VERSION = 2.0

# Baudrate
BAUDRATE = 57600


class DynamixelController():
    def __init__(self, device_name, motor_ids : list = []):
        self.motor_ids = motor_ids

        # Initialize PortHandler and PacketHandler
        self.portHandler = PortHandler(device_name)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port!")
            logger.info("Succeeded to open the port!")
        else:
            print("Failed to open the port!")
            logger.error("Failed to open the port!")
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate!")
            logger.info("Succeeded to change the baudrate!")
        else:
            print("Failed to change the baudrate!")
            logger.error("Failed to change the baudrate!")
            quit()

        self.id_to_name = {}
        self.name_to_id = {}

        self.max_motor_position = 2**32

    def get_motor_ids(self):
        return self.motor_ids
    
    def get_motor_name_from_id(self, id):
        pass

    def get_motor_id_from_name(self, name):
        pass
    
    def get_operating_mode(self, id):
        pass

    def check_ok(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            logger.error("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            logger.error("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            return True
        
    def set_operating_mode(self, id, mode=VELOCITY_CONTROL_MODE):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_OPERATING_MODE, mode)

        status_ok = self.check_ok(dxl_comm_result, dxl_error)
        if status_ok:
            logger.info(f"Operating Mode Set to {mode}")

    def set_position(self, id, position):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_GOAL_POSITION, position)
        
        status_ok = self.check_ok(dxl_comm_result, dxl_error)
        if status_ok:
            logger.info(f"Goal Position set to {position} for motor {id}")

    def set_profile_velocity(self, id, velocity):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_PROF_VELOCITY, velocity)

        status_ok = self.check_ok(dxl_comm_result, dxl_error)
        if status_ok:
            logger.info(f"Profile Velocity set to {velocity} RPM for motor {id}.")

    def get_velocity(self, id):
        dxl_present_velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, ADDR_PRESENT_VELOCITY)
        
        status_ok = self.check_ok(dxl_comm_result, dxl_error)
        if status_ok:
            logger.debug(f"Present Velocity for motor {id}: {dxl_present_velocity}")
            return dxl_present_velocity


    def set_velocity(self, id, velocity_rpm):
        # Example: Set goal velocity to 100 RPM (approx. 438 units)
        goal_velocity_unit = int(velocity_rpm / 0.229) # Convert RPM to DXL units

        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, id, ADDR_GOAL_VELOCITY, goal_velocity_unit)

        status_ok = self.check_ok(dxl_comm_result, dxl_error)
        if status_ok:
            logger.info(f"Goal velocity set to {velocity_rpm} RPM for motor {id}.")

    def get_position(self, id):
        dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, id, ADDR_PRESENT_POSITION)

        status_ok = self.check_ok(dxl_comm_result, dxl_error)
        if status_ok:
            logger.info(f"Present Position for Motor {id}: {dxl_present_position}")
        return dxl_present_position

    def set_torque(self, id, value=TORQUE_DISABLE):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, value)

        status_ok = self.check_ok(dxl_comm_result, dxl_error)
        if status_ok:
            logger.info(f"Motor {id} Torque set to {value}")

    
if __name__ == '__main__':
    controller = DynamixelController(device_name='/dev/tty.usbserial-FTAKRMAJ', motor_ids=[1, 2])

    controller.set_torque(1, TORQUE_DISABLE)
    controller.set_torque(2, TORQUE_DISABLE)

    controller.set_operating_mode(1, VELOCITY_CONTROL_MODE)
    controller.set_operating_mode(2, VELOCITY_CONTROL_MODE)

    # controller.set_operating_mode(1, POSITION_CONTROL_MODE)
    # controller.set_operating_mode(2, POSITION_CONTROL_MODE)

    controller.set_operating_mode(1, EXTENDED_POSITION_CONTROL_MODE)
    # exit()
    # controller.set_torque(1, TORQUE_ENABLE)
    # controller.set_torque(2, TORQUE_ENABLE)

    while True:
        print(f"Position For Motor 1: {controller.get_position(1)}")
        time.sleep(0.1)
    
    

    # speed = 15

    # try: 
    #     controller.set_velocity(1, speed)
    #     controller.set_velocity(2, speed)
    #     time.sleep(3)
    # except KeyboardInterrupt:
    #     pass

    # controller.set_velocity(1, 0)
    # controller.set_velocity(2, 0)


    
