"""Helpers shared by the hardware launch files: hardware selection and the
controller parameters that depend on it."""

import os
import tempfile

import xacro
import yaml


def hardware_name():
    """config/hardware/<name>.yaml, from OM_HARDWARE (default yardforce500)."""
    return os.getenv('OM_HARDWARE', 'yardforce500')


def robot_description(share_directory, worx_transport=None):
    xacro_file = os.path.join(share_directory, 'description/robot.urdf.xacro')
    mappings = {
        'use_ros2_control': '1',
        'use_sim_time': '0',
        'hardware': hardware_name(),
        'worx_transport': worx_transport or os.getenv('OM_WORX_TRANSPORT', 'spidev'),
    }
    return xacro.process_file(xacro_file, mappings=mappings).toxml()


def controller_parameters_file(share_directory):
    """controllers.yaml with wheel separation/radius taken from the hardware yaml."""
    controller_path = os.path.join(share_directory, 'config', 'controllers.yaml')
    hardware_path = os.path.join(share_directory, 'config', 'hardware', hardware_name() + '.yaml')
    with open(controller_path, 'r', encoding='utf-8') as stream:
        controllers = yaml.safe_load(stream)
    with open(hardware_path, 'r', encoding='utf-8') as stream:
        hardware = yaml.safe_load(stream)
    params = controllers.setdefault('diff_drive_base_controller', {}).setdefault('ros__parameters', {})
    params['wheel_separation'] = 2.0 * abs(float(hardware['wheel']['offset'][1]))
    params['wheel_radius'] = float(hardware['wheel']['radius'])

    params_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='openmower_controllers_', suffix='.yaml', delete=False)
    with params_file:
        yaml.safe_dump(controllers, params_file, sort_keys=False)
    return params_file.name
