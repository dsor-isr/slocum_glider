import rospy
from slocum_msgs.srv import GetFile, GetFileResponse
from threading import Event, Semaphore
import base64


class GetFileService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/get_file',
                               GetFile,
                               self.get_file)
        # Save a reference to the serial port object.
        self.ser = ser

        # Create a semaphore used to make sure that no concurrent file
        # transfers happen.
        self.transfer_semaphore = Semaphore(1)

        # Create an event used to signal when a transfer is complete.
        self.transfer_finished_event = Event()

        self.base64 = ''

    def get_file(self, req):

        file_name = req.name
        block = req.block

        rospy.lodgebug('Got request: %s, %s', file_name, block)
        acquired = self.transfer_semaphore.acquire(block)

        if not acquired:
            # If the semaphore has not been acquired then another transfer is
            # in progress and the caller has asked us to not block. Return
            # immediately.
            return GetFileResponse(success=False,
                                   message="another transfer is in progress")
        self.base64 = ''

        # If we get here, then there is no file transfer in progress. Send a
        # request to the glider to start a transfer, wait on it to finish,
        # release the semaphore, and return the result.
        self.transfer_finished_event.clear()
        # Send the request to the glider to start transferring the file here:
        self.ser.send_message('FR,'+file_name)
        # Wait for the transfer to finish
        self.transfer_finished_event.wait()

        # Release the semaphore.
        self.transfer_semaphore.release()
        # Takes the base64 message and decodes it
        s64 = base64.b64decode(self.base64).decode('utf-8')
        # Return the results.
        return GetFileResponse(success=True, contents=s64)

    def handle_serial_msg(self, msg):
        """Called when a sentence of type FI is received over the serial port.

        msg is a string starting with "FI," followed by up to 256 bytes of
        base64 encoded data.

        """

        # When the file transfer is complete, signal the waiting service
        # handler by calling
        # Change to check for stuff
        if msg == 'FI':
            return self.transfer_finished_event.set()
        else:
            self.base64 += msg[3:]
