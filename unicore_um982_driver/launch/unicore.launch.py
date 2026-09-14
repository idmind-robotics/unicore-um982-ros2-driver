#!/usr/bin/env python3
"""
@file unicore.launch.py
@brief Launch file for the Unicore UM982 GPS driver with NTRIP bridge integration

The driver node is the only process that opens the serial port. str2str acts as a
pure NTRIP -> local TCP relay: it reads RTCM corrections from the caster and writes
them to 127.0.0.1:<ntrip_local_port>. The node connects to that port and forwards
the bytes into the serial port, so host and receiver are always on the same wire.

Features:
- Single owner of the serial device (no /dev/ttyUSB* race with str2str)
- Command-line launch arguments take precedence; YAML config fills the gaps
- OpaqueFunction builds the Node after arg resolution, validating that numeric
  values are real integers so typos fail the launch with a clear message
- Conditional NTRIP bridge execution with respawn capability

Launch Arguments:
- enable_ntrip: Enable/disable NTRIP RTK corrections (default: true)
- config_file: Path to YAML parameter file
- ntrip_server, ntrip_port, ntrip_user, ntrip_pass, ntrip_mountpoint: NTRIP settings
- ntrip_local_port: Local TCP port str2str relays RTCM to (default: 40001)
- gps_port, gps_baudrate: GPS serial connection settings

@author Sonnet4
@date 2025
"""

import os
import shutil
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Default value of every launch argument that can also be supplied by the YAML config.
# When a launch argument still holds its default (i.e. it was not set on the command
# line), the corresponding YAML value is used as a fallback.
LAUNCH_ARG_DEFAULTS = {
    'ntrip_server': 'rtk2go.com',
    'ntrip_port': '2101',
    'ntrip_user': 'user',
    'ntrip_pass': 'password',
    'ntrip_mountpoint': 'FIXED',
    'ntrip_local_port': '40001',
    'gps_port': '/dev/ttyUSB0',
    'gps_baudrate': '115200',
}

# Map YAML parameter names to the launch arguments they may supply defaults for.
YAML_TO_LAUNCH_ARG = {
    'ntrip_server': 'ntrip_server',
    'ntrip_port': 'ntrip_port',
    'ntrip_user': 'ntrip_user',
    'ntrip_pass': 'ntrip_pass',
    'ntrip_mountpoint': 'ntrip_mountpoint',
    'ntrip_local_port': 'ntrip_local_port',
    'port': 'gps_port',
    'baudrate': 'gps_baudrate',
}


def load_params_and_override_args(context):
    """Fill gaps in the launch configuration from the YAML parameter file.

    Launch arguments explicitly provided by the user take precedence. A YAML
    value is applied only when the corresponding argument was left at its
    declared default value.
    """
    config_file = context.launch_configurations.get('config_file', '')
    if not os.path.exists(config_file):
        return []

    try:
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        params = config.get('unicore_um982_driver', {}).get('ros__parameters', {})

        for yaml_key, arg_key in YAML_TO_LAUNCH_ARG.items():
            if yaml_key not in params:
                continue
            if context.launch_configurations.get(arg_key) == LAUNCH_ARG_DEFAULTS[arg_key]:
                context.launch_configurations[arg_key] = str(params[yaml_key])
    except Exception as e:
        print(f"Error loading config file {config_file}: {e}")

    return []


def build_ntrip_client(context):
    """Build the str2str NTRIP -> local TCP bridge from the resolved launch arguments.

    Runs after load_params_and_override_args, so the launch configuration holds
    either the command-line value or the YAML fallback for every setting. The
    NTRIP URL and the tcpsvr output are constructed exactly once here, guaranteeing
    that the logged command matches what is actually executed.
    """
    actions = []

    if context.launch_configurations.get('enable_ntrip', 'true').lower() != 'true':
        return actions

    str2str_path = shutil.which('str2str')
    if not str2str_path:
        actions.append(LogInfo(msg="ERROR: str2str executable not found! Please install RTKLIB. "
                                    "See README.md for installation instructions."))
        return actions

    ntrip_server = context.launch_configurations['ntrip_server']
    ntrip_port = context.launch_configurations['ntrip_port']
    ntrip_user = context.launch_configurations['ntrip_user']
    ntrip_pass = context.launch_configurations['ntrip_pass']
    ntrip_mountpoint = context.launch_configurations['ntrip_mountpoint']
    ntrip_local_port = context.launch_configurations['ntrip_local_port']

    ntrip_url = f"ntrip://{ntrip_user}:{ntrip_pass}@{ntrip_server}:{ntrip_port}/{ntrip_mountpoint}"
    # str2str relays RTCM to the local TCP port; the driver node is the only
    # process that talks to the receiver, so no serial output here.
    tcp_output = f"tcpsvr://127.0.0.1:{ntrip_local_port}"

    ntrip_client = ExecuteProcess(
        cmd=[
            'str2str',
            '-in', ntrip_url,
            '-out', tcp_output,
            '-s', '5000',   # 5 second timeout (in milliseconds)
            '-r', '1000'    # 1 second reconnection interval (in milliseconds)
        ],
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        shell=False
    )

    actions.append(LogInfo(msg=f"NTRIP URL: {ntrip_url}"))
    actions.append(LogInfo(msg="Starting NTRIP -> local TCP bridge with str2str."))
    actions.append(LogInfo(msg=f"str2str command: str2str -in {ntrip_url} -out {tcp_output} -s 5000 -r 1000"))
    actions.append(ntrip_client)
    return actions


def build_driver_node(context):
    """Build the driver Node now that launch arguments are fully resolved.

    Declared here (not statically) so that gps_baudrate / ntrip_local_port /
    enable_ntrip reach the node as correctly typed parameters, while still
    letting YAML defaults flow through. Invalid numeric values abort the launch
    with a clear message instead of failing deep inside the node.
    """
    def require_int(value, name):
        try:
            return int(value)
        except (TypeError, ValueError):
            raise RuntimeError(
                f"ERROR: launch argument '{name}' must be an integer, got '{value}'. "
                "Check the value in your launch command or the YAML config file.")

    baudrate = require_int(context.launch_configurations['gps_baudrate'], 'gps_baudrate')
    if baudrate not in (9600, 115200, 230400):
        raise RuntimeError(
            f"ERROR: launch argument 'gps_baudrate' must be one of 9600, 115200, 230400, got {baudrate}.")

    ntrip_local_port = require_int(context.launch_configurations['ntrip_local_port'], 'ntrip_local_port')
    if not (1024 <= ntrip_local_port <= 65535):
        raise RuntimeError(
            f"ERROR: launch argument 'ntrip_local_port' must be between 1024 and 65535, got {ntrip_local_port}.")

    enable_ntrip = context.launch_configurations['enable_ntrip'].lower() in ('true', '1', 'on')

    node = Node(
        package='unicore_um982_driver',
        executable='unicore_um982_driver_node',
        name='unicore_um982_driver',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'port': context.launch_configurations['gps_port'],
                'baudrate': baudrate,
                'ntrip_local_port': ntrip_local_port,
                'enable_ntrip': enable_ntrip,
            },
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args',
                   '--remap', 'diagnostics:=gps/diagnostics',
                   '--log-level', LaunchConfiguration('log_level')]
    )
    return [node]


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('unicore_um982_driver')
    config_file = os.path.join(pkg_dir, 'config', 'unicore_driver_params.yaml')

    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to the parameter configuration file'
    )

    enable_ntrip_arg = DeclareLaunchArgument(
        'enable_ntrip',
        default_value='true',
        description='Enable NTRIP client (str2str) for RTK corrections'
    )

    ntrip_server_arg = DeclareLaunchArgument(
        'ntrip_server',
        default_value='rtk2go.com',
        description='NTRIP caster server hostname'
    )

    ntrip_port_arg = DeclareLaunchArgument(
        'ntrip_port',
        default_value='2101',
        description='NTRIP caster port'
    )

    ntrip_user_arg = DeclareLaunchArgument(
        'ntrip_user',
        default_value='user',
        description='NTRIP username'
    )

    ntrip_pass_arg = DeclareLaunchArgument(
        'ntrip_pass',
        default_value='password',
        description='NTRIP password'
    )

    ntrip_mountpoint_arg = DeclareLaunchArgument(
        'ntrip_mountpoint',
        default_value='FIXED',
        description='NTRIP mountpoint'
    )

    ntrip_local_port_arg = DeclareLaunchArgument(
        'ntrip_local_port',
        default_value='40001',
        description='Local TCP port that str2str relays RTCM corrections to'
    )

    gps_port_arg = DeclareLaunchArgument(
        'gps_port',
        default_value='/dev/ttyUSB0',
        description='GPS serial port device'
    )

    gps_baudrate_arg = DeclareLaunchArgument(
        'gps_baudrate',
        default_value='115200',
        description='GPS serial port baudrate (9600, 115200, 230400)'
    )

    # Add log level argument for debugging
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level for the GPS driver node (DEBUG, INFO, WARN, ERROR, FATAL)'
    )

    # Load YAML defaults to fill gaps not explicitly set on the command line
    load_yaml_params = OpaqueFunction(function=load_params_and_override_args)

    # Build the str2str NTRIP -> local TCP bridge from the resolved launch arguments
    build_ntrip = OpaqueFunction(function=build_ntrip_client)

    # Build the driver node (only owner of the serial device) from the resolved
    # launch arguments, validating numeric values as we go
    build_node = OpaqueFunction(function=build_driver_node)

    return LaunchDescription([
        config_file_arg,
        enable_ntrip_arg,
        ntrip_server_arg,
        ntrip_port_arg,
        ntrip_user_arg,
        ntrip_pass_arg,
        ntrip_mountpoint_arg,
        ntrip_local_port_arg,
        gps_port_arg,
        gps_baudrate_arg,
        log_level_arg,
        load_yaml_params,  # Fill YAML defaults first
        build_ntrip,       # Then build str2str from the resolved launch arguments
        build_node         # Finally build the node from the resolved arguments
    ])