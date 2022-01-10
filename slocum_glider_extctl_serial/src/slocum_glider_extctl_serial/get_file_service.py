import rospy
import random
from six import ensure_binary
from slocum_glider_msgs.srv import GetFile, GetFileResponse
from threading import Event, Semaphore
import base64


class FileTransferFailed(Exception):
    def __init__(self, message):
        self.message = message


class GetFileService:
    def __init__(self, ser):
        self.s = rospy.Service('extctl/get_file',
                               GetFile,
                               self.file_handler)
        # Save a reference to the serial port object.
        self.ser = ser

        # Create a semaphore used to make sure that no concurrent file
        # transfers happen.
        self.transfer_semaphore = Semaphore(1)

        # Create an event used to signal when a transfer is complete.
        self.transfer_finished_event = Event()

        self.base64 = []
        self.current_transfer_file_name = None

        # Keeping track of corrupted transfers.
        self.current_transfer_corrupted = False
        self.corrupted_transfer_counter = 0
        self.corrupted_transfer_timer = None

    def file_handler(self, req):

        file_name = req.name
        block = req.block

        try:
            message = self.get_file(file_name, block)
        except FileTransferFailed as ex:
            return GetFileResponse(success=False, message=ex.message)

        return GetFileResponse(success=True, contents=message)

    def get_file(self, file_name, block):
        rospy.logdebug('Got request: %s, %s', file_name, block)
        acquired = self.transfer_semaphore.acquire(block)

        if not acquired:
            # If the semaphore has not been acquired then another transfer is
            # in progress and the caller has asked us to not block.
            raise FileTransferFailed('another transfer is in progress')

        self.base64 = []
        file_name = ensure_binary(file_name)
        self.current_transfer_file_name = file_name
        self.current_transfer_corrupted = False
        self.corrupted_transfer_counter = 0
        self.corrupted_transfer_timer = None

        # If we get here, then there is no file transfer in progress. Send a
        # request to the glider to start a transfer, wait on it to finish,
        # release the semaphore, and return the result.
        self.transfer_finished_event.clear()
        # Send the request to the glider to start transferring the file here:
        self.ser.send_message(b'FR,' + file_name)
        # Wait for the transfer to finish
        self.transfer_finished_event.wait()

        # Release the semaphore.
        self.transfer_semaphore.release()

        # Check if we were successful
        if self.current_transfer_corrupted:
            raise FileTransferFailed('transfer corrupted')

        # Takes the base64 message and decodes it
        s64 = b''.join(map(base64.b64decode, self.base64)).decode('utf-8')
        # Return the results.
        return s64

    def retry_get_file_unsafe(self, event):
        rospy.logwarn('A file transfer was corrupted, but the finish message '
                      'was never seen. Hoping that the glider thinks the '
                      'file transfer is finished and retrying with fingers '
                      'crossed.')
        self.retry_get_file(event)

    def retry_get_file(self, event):
        self.corrupted_transfer_counter += 1
        rospy.logwarn('Retrying file transfer of %s. This is attempt %s',
                      self.current_transfer_file_name,
                      self.corrupted_transfer_counter + 1)
        if self.corrupted_transfer_counter == 4:
            # We've tried four times and can't get this file... Just abort.
            self.transfer_finished_event.set()
        else:
            # Try again!
            self.base64 = []
            self.current_transfer_corrupted = False
            self.ser.send_message(b'FR,' + self.current_transfer_file_name)

    def handle_serial_msg(self, msg):
        """Called when a sentence of type FI is received over the serial port.

        msg is a string starting with "FI," followed by up some number of bytes
        of base64 encoded data.

        """

        # When the file transfer is complete, signal the waiting service
        # handler by calling
        # Change to check for stuff
        if msg == b'FI':
            self.transfer_finished_event.set()
        elif msg.startswith(b'$FI'):
            # This is an invalid message... Ignore it until the final message
            # comes in then retry the transfer.
            self.current_transfer_corrupted = True
            # Wait until the finish message is found somewhere or 4 seconds has
            # passed to retry the transfer.
            timer = self.corrupted_transfer_timer
            if timer:
                timer.shutdown()
            if b'$FI*0f' in msg:
                # The glider thinks the file transfer has finished. We can
                # request that a new one starts at any point. We wait a random
                # amount of time in order to try and avoid hitting the glider
                # in the same part of its control loop every time.
                timer = rospy.Timer(rospy.Duration(random.uniform(1, 4)),
                                    self.retry_get_file,
                                    oneshot=True)
            else:
                # We don't have a file transfer end notification yet, wait a
                # couple seconds for one before retrying.
                timer = rospy.Timer(rospy.Duration(4),
                                    self.retry_get_file_unsafe,
                                    oneshot=True)
            self.corrupted_transfer_timer = timer
        else:
            # We push to a list of base64 messages instead of concatenating
            # them into one big byte string because it's possible that padding
            # requirements will result in an = character in the
            # message. Python's base64 decoder will then stop decoding once it
            # hits the first =.
            self.base64.append(msg[3:])
