import serial
import time
import struct
from typing import List

from rclpy.clock import Clock
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from ros2_control_interfaces.msg import HardwareInterfaceState

class RaspberryPiPicoHardwareInterface:
    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port=port, baudrate=baudrate)
        self.ser.flushInput()

    def read(self, size: int) -> bytes:
        return self.ser.read(size)

    def write(self, data: bytes) -> None:
        self.ser.write(data)

    def read_floats(self, count: int) -> List[float]:
        return struct.unpack('<' + 'f' * count, self.read(4 * count))

    def write_floats(self, values: List[float]) -> None:
        self.write(struct.pack('<' + 'f' * len(values), *values))

    def read_ints(self, count: int) -> List[int]:
        return struct.unpack('<' + 'i' * count, self.read(4 * count))

    def write_ints(self, values: List[int]) -> None:
        self.write(struct.pack('<' + 'i' * len(values), *values))

    def read_uints(self, count: int) -> List[int]:
        return struct.unpack('<' + 'I' * count, self.read(4 * count))

    def write_uints(self, values: List[int]) -> None:
        self.write(struct.pack('<' + 'I' * len(values), *values))

    def read_hardware_interface_state(self) -> HardwareInterfaceState:
        self.write(b'\x01')
        timestamp = Time(clock=Clock(clock_type=ClockType.STEADY_TIME)).to_msg()
        position = self.read_floats(1)[0]
        velocity = self.read_floats(1)[0]
        effort = self.read_floats(1)[0]
        state = HardwareInterfaceState(
            timestamp=timestamp,
            data=[
                HardwareInterfaceState(name='position', value=position),
                HardwareInterfaceState(name='velocity', value=velocity),
                HardwareInterfaceState(name='effort', value=effort),
            ],
        )
        return state

    def write_hardware_interface_command(self, position: float, velocity: float, effort: float) -> None:
        self.write(b'\x02')
        self.write_floats([position, velocity, effort])

    def configure(self) -> None:
        self.write(b'\x03')
        # TODO: add any configuration code here

    def cleanup(self) -> None:
        self.write(b'\x04')
        # TODO: add any cleanup code here


def main():
    node = rclpy.create_node('raspberry_pi_pico_hardware_interface')
    hw_interface = RaspberryPiPicoHardwareInterface('/dev/ttyACM0')

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        depth=10)

    depth_qos = node.create_subscription(
        HardwareInterfaceState,
        'state',
        lambda msg: hw_interface.write_hardware_interface_command(msg.data[0].value, msg.data[1].value, msg.data[2].value),
        qos_profile)

    pub = node.create_publisher(HardwareInterfaceState, 'state', qos_profile)

    def timer_callback():
        state = hw_interface.read_hardware_interface_state()
        pub.publish(state)

    timer_period = 0.1  # seconds
    timer = node.create_timer(timer_period, timer_callback)

    hw_interface.configure()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    hw_interface.cleanup()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# Here's a brief explanation of the code:

# The RaspberryPiPicoHardwareInterface class provides methods for reading and writing data over a serial connection between the Raspberry Pi 4 and Raspberry Pi Pico. The read_hardware_interface_state method reads position, velocity, and effort values from the Pico, and returns them as a HardwareInterfaceState message. The write_hardware_interface_command method writes position, velocity, and effort values to the Pico. The configure and cleanup methods can be used for any necessary initialization and cleanup code, respectively.

# In the main function, a ROS 2 node is created, along with a RaspberryPiPicoHardwareInterface instance. A subscription is created to listen for HardwareInterfaceState messages published by the ROS 2 controller, and a publisher is created to publish HardwareInterfaceState messages read from the Pico. A timer is created to periodically read the hardware interface state and publish it to the ROS 2 controller. Finally, the RaspberryPiPicoHardwareInterface is configured and the node is started using rclpy.spin.

# The depth_qos variable holds the subscription, which is created with a callback function that sends the hardware interface command to the Pico using write_hardware_interface_command.

# The timer_callback function reads the hardware interface state from the Pico using read_hardware_interface_state, creates a HardwareInterfaceState message, and publishes it to the ROS 2 controller using pub.publish(state).

# The timer_period variable sets the interval at which the timer callback function is called.

# The try/except block catches the KeyboardInterrupt exception, which is raised when the user presses Ctrl+C, and cleans up the hardware interface and ROS 2 node before exiting.

# This code provides a basic implementation of a hardware interface for ROS 2 control between a Raspberry Pi 4 and a Raspberry Pi Pico, but you may need to modify it to suit your specific requirements.
