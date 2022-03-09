##
## Severity level constants (same as rosgraph_msgs/Log)
##
byte DEBUG=1 #debug level
byte INFO=2  #general level
byte WARN=4  #warning level
byte ERROR=8 #error level
byte FATAL=16 #fatal/critical level

##
## Fields
##
Header header
byte level
string msg
string name
diagnostic_msgs/KeyValue[] extra
