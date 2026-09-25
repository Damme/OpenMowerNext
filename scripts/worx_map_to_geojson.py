#!/usr/bin/env python3
"""Convert an open_mower_ros map (map.bag written by mower_map_service) to an
OpenMowerNext GeoJSON map (OM_MAP_PATH).

ROS1 map coordinates are UTM(point) - UTM(datum) (xbot_driver_gps, grid-aligned).
OpenMowerNext uses a local ENU frame around the same datum, so points are taken
back to latitude/longitude through UTM; the map lines up with GPS in both.

    worx_map_to_geojson.py map.bag map.geojson --datum 57.12345 12.34567

Mapping:
    mowing_areas[i].area        -> Polygon, type "operation", id "mow_<i>"
    mowing_areas[i].obstacles   -> Polygon, type "exclusion", id "mow_<i>_obstacle_<j>"
    navigation_areas[i].area    -> Polygon, type "navigation", id "nav_<i>"
    docking_point               -> LineString [port, 0.5 m out of the dock], type "docking_station"

The ROS1 docking point is the robot's base_link pose when docked (facing the
station). OpenMowerNext stores a docking station as the charging port pose
rotated by 180 deg (map_recorder); docking_helper subtracts the
base_link->charging_port offset again, so --charging-port-offset must match the
URDF (config/hardware/<hw>.yaml: chassis offset x + length; Worx 0.47).

Needs: pip install rosbags pyproj
"""

import argparse
import json
import math
import sys

try:
    from pyproj import Proj
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg
except ImportError as e:  # pragma: no cover
    sys.exit(f'missing dependency ({e}); run: pip install rosbags pyproj')

MAP_AREA_MSG = """string name
geometry_msgs/Polygon area
geometry_msgs/Polygon[] obstacles
"""


def utm_zone(lat, lon):
    """Zone as GeographicLib UTMUPS (zonespec MATCH/STANDARD) incl. Norway/Svalbard."""
    zone = int((lon + 180.0) // 6.0) + 1
    if 56.0 <= lat < 64.0 and 3.0 <= lon < 12.0:
        zone = 32
    if 72.0 <= lat < 84.0 and lon >= 0.0:
        if lon < 9.0:
            zone = 31
        elif lon < 21.0:
            zone = 33
        elif lon < 33.0:
            zone = 35
        elif lon < 42.0:
            zone = 37
    return zone


class Ros1ToLatLon:
    def __init__(self, datum_lat, datum_lon):
        self.proj = Proj(proj='utm', zone=utm_zone(datum_lat, datum_lon), ellps='WGS84',
                         south=datum_lat < 0)
        self.e0, self.n0 = self.proj(datum_lon, datum_lat)

    def __call__(self, x, y):
        lon, lat = self.proj(self.e0 + x, self.n0 + y, inverse=True)
        return [lon, lat]


def ring(points, conv):
    coords = [conv(p.x, p.y) for p in points]
    if coords and coords[0] != coords[-1]:
        coords.append(coords[0])
    return coords


def feature(fid, name, ftype, geometry):
    return {'type': 'Feature',
            'properties': {'id': fid, 'name': name, 'type': ftype},
            'geometry': geometry}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('out')
    ap.add_argument('--datum', nargs=2, type=float, required=True, metavar=('LAT', 'LON'),
                    help='OM_DATUM_LAT OM_DATUM_LONG of the ROS1 robot')
    ap.add_argument('--charging-port-offset', type=float, default=0.47,
                    help='base_link -> charging_port distance along x in the URDF (default 0.47, Worx)')
    args = ap.parse_args(argv)

    typestore = get_typestore(Stores.ROS1_NOETIC)
    types = get_types_from_msg(MAP_AREA_MSG, 'mower_map/msg/MapArea')
    typestore.register(types)
    conv = Ros1ToLatLon(*args.datum)

    features, counts = [], {'mowing_areas': 0, 'navigation_areas': 0, 'docking_point': 0, 'obstacles': 0}
    with Reader(args.bag) as reader:
        for conn, _, raw in reader.messages():
            msg = typestore.deserialize_ros1(raw, conn.msgtype)
            if conn.topic in ('mowing_areas', 'navigation_areas'):
                mowing = conn.topic == 'mowing_areas'
                i = counts[conn.topic]
                counts[conn.topic] += 1
                prefix = 'mow' if mowing else 'nav'
                name = msg.name or (f'Mowing area {i + 1}' if mowing else f'Navigation area {i + 1}')
                features.append(feature(f'{prefix}_{i}', name, 'operation' if mowing else 'navigation',
                                        {'type': 'Polygon', 'coordinates': [ring(msg.area.points, conv)]}))
                for j, obstacle in enumerate(msg.obstacles):
                    if len(obstacle.points) < 3:
                        continue
                    counts['obstacles'] += 1
                    features.append(feature(f'{prefix}_{i}_obstacle_{j}', f'{name} obstacle {j + 1}', 'exclusion',
                                            {'type': 'Polygon', 'coordinates': [ring(obstacle.points, conv)]}))
            elif conn.topic == 'docking_point':
                counts['docking_point'] += 1
                p, q = msg.position, msg.orientation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                off = args.charging_port_offset
                port = (p.x + off * math.cos(yaw), p.y + off * math.sin(yaw))
                out_yaw = yaw + math.pi  # dock pose faces out of the station
                ahead = (port[0] + 0.5 * math.cos(out_yaw), port[1] + 0.5 * math.sin(out_yaw))
                features.append(feature('dock_0', 'Docking station', 'docking_station',
                                        {'type': 'LineString',
                                         'coordinates': [conv(*port), conv(*ahead)]}))

    with open(args.out, 'w', encoding='utf-8') as f:
        json.dump({'type': 'FeatureCollection', 'features': features}, f, indent=2)
    print(f"wrote {args.out}: {counts['mowing_areas']} mowing, {counts['navigation_areas']} navigation, "
          f"{counts['obstacles']} obstacles, {counts['docking_point']} docking point(s)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
