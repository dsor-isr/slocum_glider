from contextlib import closing
from os.path import splitext

import rospy

from slocum_glider_msgs.srv import GetFile, GetFileResponse


class GetFileService:
    """Implement the extctl/get_file service for retrieving files from the glider's
science computer.

    """
    def __init__(self, science_fs):
        self.s = rospy.Service('extctl/get_file',
                               GetFile,
                               self.file_handler)
        self.science_fs = science_fs

    def file_handler(self, req):

        file_name = req.name

        root, ext = splitext(file_name)

        if '/' in root \
           or '.' in root \
           or len(root) > 8 \
           or len(ext) < 2 \
           or len(ext) > 4:
            rospy.logwarn('File name failed validation: %s', file_name)
            return GetFileResponse(success=False, message='file name invalid')

        rospy.loginfo('Reading file %s', file_name)
        with closing(self.science_fs.open([b'config', file_name], 'rb')) as f:
            return GetFileResponse(success=True, contents=f.read())
