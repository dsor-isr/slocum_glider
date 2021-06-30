from contextlib import closing
from os.path import splitext

import rospy

from slocum_glider_msgs.srv import SendFile, SendFileResponse


class SendFileService:
    """Implement the extctl/send_file service for saving files from to glider's
science computer.

    """
    def __init__(self, science_fs):
        self.s = rospy.Service('extctl/send_file',
                               SendFile,
                               self.send_file)
        self.science_fs = science_fs

    def send_file(self, req):

        file_name = req.name
        contents = req.contents

        root, ext = splitext(file_name)

        if '/' in root \
           or '.' in root \
           or len(root) > 8 \
           or len(ext) < 2 \
           or len(ext) > 4:
            rospy.logwarn('File name failed validation: %s', file_name)
            return SendFileResponse(success=False)

        rospy.loginfo('Writing file %s', file_name)
        with closing(self.science_fs.open([b'logs', file_name], 'wb')) as f:
            f.write(contents)

        return SendFileResponse(success=True)
