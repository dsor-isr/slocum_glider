import rospy

from six import ensure_binary
from slocum_glider_msgs.srv import SendFile, SendFileResponse
from threading import Semaphore
import base64


class SendFileService:
    def __init__(self, ser):
        # Save a reference to the serial port object.
        self.ser = ser

        # Create a semaphore used to make sure that no concurrent file
        # transfers happen.
        self.transfer_semaphore = Semaphore(1)

        self.s = rospy.Service('extctl/send_file',
                               SendFile,
                               self.send_file)

    def send_file(self, req):

        file_name = ensure_binary(req.name)
        block = req.block
        contents = req.contents

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
        self.ser.send_message(b'FW,'+file_name)

        # Encode
        s64 = base64.b64encode(contents)
        while s64 != b'':
            self.ser.send_message(b'FO,'+s64[:256])
            s64 = s64[256:]
            # TODO: adjust this sleep time based on testing.
            rospy.sleep(0.1)

        self.ser.send_message(b'FC')

        # Release the semaphore.
        self.transfer_semaphore.release()
        return SendFileResponse(success=True)
