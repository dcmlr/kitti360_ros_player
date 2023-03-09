# This message contains the instanceID for each pixel of a camera

uint32 height  # height of the image
uint32 width   # width of the image
uint16[] data  # array of length height*width where each cell contains the 16-bit instanceID
