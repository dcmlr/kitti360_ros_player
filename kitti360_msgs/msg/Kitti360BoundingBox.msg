# based on what I have seen, uint32 for the index should definitely be large
# enough. 
uint32 index
string label
string category
float32 level_min
float32 level_max
bool dynamic
uint32 dynamic_seq
uint32 dynamic_idx

# transform to the map/world frame
std_msgs/Float32MultiArray transform
# vertices that the bounding box define
std_msgs/Float32MultiArray vertices
# each Vector describes one surface defined by three points which are
# referenced by index from `vertices`
std_msgs/Int16MultiArray faces
