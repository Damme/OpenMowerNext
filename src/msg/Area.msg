# Definition of the area
# The area is defined by a polygon
# The area can be of 3 types:
# - Exclusion: the area is forbidden
# - Navigation: the area is a navigation area
# - Operation: the area is an operation area (e.g. lawn mowing area)

uint8 TYPE_EXCLUSION = 0
uint8 TYPE_NAVIGATION = 1
uint8 TYPE_OPERATION = 2

string id # id of the area
string name # name of the area
uint8 type # type of the area. 0 = obstacle, 1 = navigation, 2 = operation
geometry_msgs/PolygonStamped area # area of the zone
# Poses (map frame) the mower drove while the boundary was recorded, every few cm
# and while turning in place. The whole body stood there, so it may overhang the
# line that far (map_server grid.stance_*). Empty: unknown (derived from the polygon).
geometry_msgs/Pose2D[] stance
# Mowing direction for an operation area (coverage_server, serpentine fill only). The lanes run
# at mow_angle degrees in the map frame (counter-clockwise from +x / east, 0..360), the first one
# in that direction. < 0: automatic (the coverage_server's optimize_sweep_angle, else the
# request's swath_angle). Overrides optimize_sweep_angle when set.
float64 mow_angle -1.0
# true: the same lanes, each driven the other way (also with the automatic angle)
bool mow_reverse false
