#!/usr/bin/env python3
"""Build-time generation of what the python launch files compute at start-up,
so the robot can run without python (om_launch manifests):

  <out>/worx_<transport>.urdf        robot_description (xacro), transport spidev | fake
  <out>/worx_controllers.yaml        controllers.yaml + wheel geometry from config/hardware/worx.yaml
  <out>/worx_controllers_sim.yaml    same, diff drive publishing odom -> base_link (kinematic sim)

Usage: om_generate.py <package source dir> <output dir>
"""
import os
import sys
import tempfile

import xacro
import yaml


def main():
    src, out = sys.argv[1], sys.argv[2]
    os.makedirs(out, exist_ok=True)

    # $(find open_mower_next) in the xacro files: point a throw-away package index at the sources.
    prefix = tempfile.mkdtemp(prefix='om_generate_')
    os.makedirs(os.path.join(prefix, 'share', 'ament_index', 'resource_index', 'packages'))
    open(os.path.join(prefix, 'share', 'ament_index', 'resource_index', 'packages', 'open_mower_next'), 'w').close()
    os.symlink(os.path.abspath(src), os.path.join(prefix, 'share', 'open_mower_next'))
    os.environ['AMENT_PREFIX_PATH'] = prefix + os.pathsep + os.environ.get('AMENT_PREFIX_PATH', '')

    # Environment the xacro files read stays a runtime choice: om_container
    # expands $(env ...) when it reads the URDF (robot_description@file).
    runtime_env = {'OM_WORX_MOTORS_ENABLED': 'true', 'OM_WORX_BLADE_ENABLED': 'true'}
    for name in runtime_env:
        os.environ[name] = f'@@{name}@@'
    for transport in ('spidev', 'fake'):
        urdf = xacro.process_file(os.path.join(src, 'description', 'robot.urdf.xacro'), mappings={
            'use_ros2_control': '1', 'use_sim_time': '0', 'hardware': 'worx', 'worx_transport': transport,
        }).toxml()
        for name, default in runtime_env.items():
            urdf = urdf.replace(f'@@{name}@@', f'$(env {name} {default})')
        with open(os.path.join(out, f'worx_{transport}.urdf'), 'w', encoding='utf-8') as f:
            f.write(urdf)

    # Same as launch/hardware_common.py controller_parameters_file().
    with open(os.path.join(src, 'config', 'controllers.yaml'), encoding='utf-8') as f:
        controllers = yaml.safe_load(f)
    with open(os.path.join(src, 'config', 'hardware', 'worx.yaml'), encoding='utf-8') as f:
        hardware = yaml.safe_load(f)
    params = controllers.setdefault('diff_drive_base_controller', {}).setdefault('ros__parameters', {})
    params['wheel_separation'] = 2.0 * abs(float(hardware['wheel']['offset'][1]))
    params['wheel_radius'] = float(hardware['wheel']['radius'])
    with open(os.path.join(out, 'worx_controllers.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(controllers, f, sort_keys=False)
    params['enable_odom_tf'] = True
    with open(os.path.join(out, 'worx_controllers_sim.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(controllers, f, sort_keys=False)


if __name__ == '__main__':
    main()
