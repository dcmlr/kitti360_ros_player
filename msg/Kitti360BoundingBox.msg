# based on what I have seen, uint32 for the index should definitely be large
# enough. 
uint32 index
string label
string category
float32 level_min
float32 level_max
bool dynamic
uint32 dynamicSeq
uint32 dynamicIdx

# vertices that the bounding box define
geometry_msgs/Point32[] vertices
# each Vector describes one surface defined by three points which are
# referenced by index from `vertices`
geometry_msgs/Vector3[] faces
