# This message contains the confidence for each pixel of a camera

uint32 height  # height of the image
uint32 width   # width of the image
uint16[] data  # array of length height*width where each cell contains a 16-bit confidence value
