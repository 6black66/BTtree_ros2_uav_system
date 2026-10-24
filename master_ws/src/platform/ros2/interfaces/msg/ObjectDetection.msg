# Coordinates in original image pixels: top-left x/y, width/height.
# target_id persists across adjacent slow frames using class + IoU association;
# it is not a globally unique identity or a re-identification guarantee.
uint64 target_id
int32 class_id
string label
float32 confidence
float32 x
float32 y
float32 width
float32 height
