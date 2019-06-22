uint16 TYPE_STOP      = 0
uint16 TYPE_LEFT      = 1
uint16 TYPE_RIGHT     = 2
uint16 TYPE_PED       = 3
uint16 TYPE_CURVY     = 4
uint16 TYPE_PASS_LEFT = 5

uint16 type
float32 distance
std_msgs/Time lastSeen
geometry_msgs/Point position
geometry_msgs/Point size