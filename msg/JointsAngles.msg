# The header specifies the time at which the joint states were recorded. 
# Each joint is uniquely identified by its name.  
# All the joint states in one message have to be recorded at the same time.
std_msgs/msg/Header header
string[] name
double[] position
