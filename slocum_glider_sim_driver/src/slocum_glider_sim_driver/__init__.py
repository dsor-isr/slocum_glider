from threading import Thread

import rospy
from std_msgs.msg import String

from .get_file_service import GetFileService
from .glider import Glider
from .glider_files import FilesystemDriver
from .ros_cmd import RosCmdTopic
from .ros_sensors import RosSensorsTopic
from .send_file_service import SendFileService


class SimDriver:
    """A driver that exposes the glider extctl control interface over ROS. The
driver connects to a software simulated glider in Gazebo or similar.

    """

    def __init__(self, flight_computer_dir, science_computer_dir,
                 masterdata_file_path):
        self.flight_fs = FilesystemDriver(flight_computer_dir)
        self.science_fs = FilesystemDriver(science_computer_dir)
        self.masterdata_file_path = masterdata_file_path

        self.ros_cmd_topic = RosCmdTopic()
        self.ros_sensors_topic = RosSensorsTopic()
        self.log_pub = rospy.Publisher('console', String, queue_size=10)

        self.glider = Glider(
            self.flight_fs,
            self.science_fs,
            masterdata_file_path,
            command_cb=self.ros_cmd_topic.send_command,
            update_state_cb=self.ros_sensors_topic.update_state,
            console_writer=self.log
        )

        self.command_sub = rospy.Subscriber('console_cmd',
                                            String,
                                            self.ros_cmd)

        # Start the file reading service.
        self.get_file_service = GetFileService(self.science_fs)

        # Start the file writing service
        self.send_file_service = SendFileService(self.science_fs)

    def log(self, msg):
        self.log_pub.publish(msg)
        rospy.loginfo(msg)

    def ros_cmd(self, msg):
        self.glider.cmd(msg.data)

    def start(self):
        """Start all processes requiring a separate thread."""
        self.glider_thread = Thread(target=self.glider.run)
        self.glider_thread.start()

    def stop(self):
        self.glider.stop_flag = True
