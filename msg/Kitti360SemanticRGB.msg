# This message contains color coded semantic labels for each pixel of a camera
# --> 3-channel 8bit

uint32 height  # height of the image
uint32 width   # width of the image
uint8[] data   # array of length 3*height*width where three values represent the semantic color of a pixel
