from json import dumps
import rospy

from rosgraph_msgs.msg import Log
from slocum_glider_msgs.msg import Log as MyLog
from slocum_glider_msgs.srv import SendFileRequest


class Logger:
    def __init__(self, service, min_level=Log.WARN):
        self.min_level = min_level
        self.service = service

        self.rosout_sub = rospy.Subscriber('/rosout_agg',
                                           Log,
                                           self.rosout_handler)
        self.mylog_sub = rospy.Subscriber('extctl/log',
                                          MyLog,
                                          self.mylog_handler)

    def rosout_handler(self, msg):
        if msg.level < self.min_level:
            return
        # Assemble the message.
        m = {
            'type': 'rosout',
            'level': msg.level,
            'sec': msg.header.stamp.sec,
            'nsec': msg.header.stamp.nsec,
            'name': msg.name,
            'msg': msg.msg,
            'file': msg.file,
            'function': msg.function,
            'line': msg.line
        }

        s = dumps(m, separators=(',', ':'))
        service.send_file(
            SendFileRequest(
                file_name='bsd.log',
                block=True,
                contents=s.encode('utf-8')
            )
        )

    def mylog_handler(self, msg):
        if msg.level < self.min_level:
            return
        # Assemble the message.
        extra = {e.key: e.value for e in msg.extra}
        m = {
            'type': 'log',
            'level': msg.level,
            'sec': msg.header.stamp.sec,
            'nsec': msg.header.stamp.nsec,
            'name': msg.name,
            'msg': msg.msg,
            'extra': {e.key: e.value for e in msg.extra}
        }

        s = dumps(m, separators=(',', ':'))
        service.send_file(
            SendFileRequest(
                file_name='bsd.log',
                block=True,
                contents=s.encode('utf-8')
            )
        )
