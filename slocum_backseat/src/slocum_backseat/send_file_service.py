import rospy

from slocum_msgs.srv import SendFile, SendFileResponse
from threading import Semaphore
import base64


class SendFileService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/send_file',
                               SendFile,
                               self.send_file)
        # Save a reference to the serial port object.
        self.ser = ser

        # Create a semaphore used to make sure that no concurrent file
        # transfers happen.
        self.transfer_semaphore = Semaphore(1)

    def send_file(self, req):

        file_name = req.name
        block = req.block
        contents = req.content

        rospy.logwarn('Got request: %s, %s', file_name, block)
        acquired = self.transfer_semaphore.acquire(block)

        if not acquired:
            # If the semaphore has not been acquired then another transfer is
            # in progress and the caller has asked us to not block. Return
            # immediately.
            return SendFileResponse(success=False, message="another transfer is in progress")

        # If we get here, then there is no file transfer in progress. Send a
        # request to the glider to start a transfer, wait on it to finish,
        # release the semaphore, and return the result.
        # Send the request to the glider to start transferring the file from here:
        self.ser.send_message('FW,'+file_name)

        # Encode
        s64 = str(base64.b64encode(contents.encode()))
        while s64 != '':
            self.ser.send_message('FO,'+s64[:256])
            s64 = s64[256:]
            # TODO: adjust this sleep time based on testing.
            rospy.sleep(0.25)

        self.ser.send_message('FC')

        # Release the semaphore.
        self.transfer_semaphore.release()
        return SendFileResponse(success=True)
