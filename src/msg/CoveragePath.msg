# One pass of a coverage plan, as produced by coverage_server.
uint8 KIND_PERIMETER = 0  # boundary loops (outermost pass rides the recorded edge)
uint8 KIND_OBSTACLE = 1   # rings around an exclusion inside the area
uint8 KIND_FILL = 2       # serpentine lanes or concentric rings

uint8 kind
bool is_outline           # perimeter/obstacle loop: follow precisely, blade on the edge
nav_msgs/Path path
