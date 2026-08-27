#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time


SESSION = 'ratgun'
WORKSPACE = Path(__file__).resolve().parent
ROS_SETUP = Path('/opt/ros/humble/setup.bash')
TROS_SETUP = Path('/opt/tros/humble/setup.bash')
INSTALL_SETUP = WORKSPACE / 'install/setup.bash'
LOCAL_SETUP = WORKSPACE / 'install/local_setup.bash'

APRIL_CAMERA = (
    'ros2 launch hobot_stereonet '
    'stereonet_model_web_visual_v2.4.launch.py '
    'stereonet_pub_web:=False '
    'publish_pcd_enabled:=False '
    'publish_visual_enabled:=False '
    'publish_origin_enable:=False '
    'publish_rectify_bgr:=True '
    'mipi_image_width:=640 '
    'mipi_image_height:=352 '
    'mipi_sub_image_width:=640 '
    'mipi_sub_image_height:=352 '
    'mipi_image_framerate:=30.0 '
    'mipi_gdc_enable:=True '
    'log_level:=warn'
)

RED_POINT_CAMERA = (
    'ros2 launch hobot_stereonet '
    'stereonet_model_web_visual_v2.4.launch.py '
    'stereonet_pub_web:=False '
    'publish_pcd_enabled:=True '
    'publish_visual_enabled:=False '
    'publish_origin_enable:=False '
    'publish_rectify_bgr:=True '
    'mipi_image_width:=640 '
    'mipi_image_height:=352 '
    'mipi_sub_image_width:=640 '
    'mipi_sub_image_height:=352 '
    'mipi_image_framerate:=30.0 '
    'mipi_gdc_enable:=True '
    'pointcloud_downsample_step:=2 '
    'pointcloud_depth_max:=4.0 '
    'pcl_filter_enable:=False '
    'speckle_filter_enable:=False '
    'save_result_flag:=False '
    'log_level:=warn'
)


def tmux(*args, check=True, capture=False):
    return subprocess.run(
        ['tmux', *args],
        check=check,
        capture_output=capture,
        text=True,
    )


def session_exists():
    return tmux(
        'has-session', '-t', f'={SESSION}', check=False, capture=True
    ).returncode == 0


def ros_command(setup_files, command):
    commands = [f'cd {shlex.quote(str(WORKSPACE))}']
    commands += [f'source {shlex.quote(str(path))}' for path in setup_files]
    commands.append(f'exec {command}')
    return ' && '.join(commands)


def create_window(name, command, first=False):
    if first:
        tmux(
            'new-session', '-d', '-s', SESSION, '-n', name,
            'bash', '-lc', command,
        )
    else:
        tmux(
            'new-window', '-t', f'={SESSION}', '-n', name,
            'bash', '-lc', command,
        )


def stop_stack():
    if not session_exists():
        print(f"No tmux session named '{SESSION}' is running.")
        return

    panes = tmux(
        'list-panes', '-s', '-t', f'={SESSION}',
        '-F', '#{pane_id}', capture=True,
    ).stdout.splitlines()

    print(f"Stopping tmux session '{SESSION}'...")
    for pane in panes:
        tmux('send-keys', '-t', pane, 'C-c', check=False)

    time.sleep(5)
    if session_exists():
        tmux('kill-session', '-t', f'={SESSION}', check=False)
    print('Ratgun stack stopped.')


def validate(mode):
    if os.geteuid() != 0:
        raise RuntimeError('Run "sudo -i" before starting the stack.')
    if shutil.which('tmux') is None:
        raise RuntimeError('tmux is not installed on the RDK100.')

    required = [ROS_SETUP, INSTALL_SETUP]
    if mode:
        required += [TROS_SETUP, LOCAL_SETUP]

    missing = [path for path in required if not path.is_file()]
    if missing:
        paths = '\n'.join(f'  {path}' for path in missing)
        raise RuntimeError(f'Required setup files are missing:\n{paths}')


def start_stack(mode):
    validate(mode)
    if session_exists():
        raise RuntimeError(
            f"Session '{SESSION}' already exists. Run ./run_stack.py --stop."
        )

    core_setup = [ROS_SETUP, INSTALL_SETUP]
    target_setup = [TROS_SETUP, LOCAL_SETUP]

    windows = [
        ('tf_tree', ros_command(
            core_setup,
            'ros2 launch tf_tree_pkg tf_tree.launch.py',
        )),
        ('closed_loop', ros_command(
            core_setup,
            'ros2 launch closed_loop_pkg closed_loop.launch.py',
        )),
        ('arduino', ros_command(
            core_setup,
            'ros2 run arduino_ros_bridge arduino_ros_bridge '
            '--ros-args -p port_name:=/dev/ttyACM0',
        )),
    ]

    if mode == 'april_tags':
        windows += [
            ('camera', ros_command([TROS_SETUP], APRIL_CAMERA)),
            ('target', ros_command(
                target_setup,
                'ros2 launch target_tf_pkg april_tags.launch.py',
            )),
        ]
    elif mode == 'red_point':
        windows += [
            ('camera', ros_command([TROS_SETUP], RED_POINT_CAMERA)),
            ('target', ros_command(
                target_setup,
                'ros2 launch target_tf_pkg '
                'coordinate_publisher_ordered.launch.py',
            )),
        ]

    try:
        for index, (name, command) in enumerate(windows):
            create_window(name, command, first=index == 0)
            if index == 0:
                tmux(
                    'set-option', '-t', f'={SESSION}',
                    'remain-on-exit', 'on',
                )
        tmux('select-window', '-t', f'={SESSION}:tf_tree')
    except subprocess.CalledProcessError:
        if session_exists():
            tmux('kill-session', '-t', f'={SESSION}', check=False)
        raise

    print(f"Started tmux session '{SESSION}'.")
    os.execvp('tmux', ['tmux', 'attach-session', '-t', f'={SESSION}'])


def parse_args():
    parser = argparse.ArgumentParser(
        description='Launch the Ratgun ROS 2 stack on the RDK100.'
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--april_tags', action='store_true')
    modes.add_argument('--red_point', action='store_true')
    modes.add_argument('--stop', action='store_true')
    return parser.parse_args()


def main():
    args = parse_args()
    if args.stop:
        stop_stack()
    elif args.april_tags:
        start_stack('april_tags')
    elif args.red_point:
        start_stack('red_point')
    else:
        start_stack(None)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f'run_stack.py: {error}', file=sys.stderr)
        sys.exit(1)
