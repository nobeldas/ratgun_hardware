#!/usr/bin/env python3

import argparse
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import time


SESSION_NAME = 'ratgun'
ARDUINO_PORT = '/dev/ttyACM0'
SHUTDOWN_WAIT_SECONDS = 5


def run_tmux(*args, check=True, capture_output=False):
    return subprocess.run(
        ['tmux', *args],
        check=check,
        capture_output=capture_output,
        text=True,
    )


def session_exists():
    result = run_tmux(
        'has-session',
        '-t',
        f'={SESSION_NAME}',
        check=False,
        capture_output=True,
    )
    return result.returncode == 0


def stop_stack():
    if not session_exists():
        print(f"No tmux session named '{SESSION_NAME}' is running.")
        return

    result = run_tmux(
        'list-panes',
        '-s',
        '-t',
        f'={SESSION_NAME}',
        '-F',
        '#{pane_id}',
        capture_output=True,
    )

    print(f"Stopping tmux session '{SESSION_NAME}'...")
    for pane_id in result.stdout.splitlines():
        pane_id = pane_id.strip()
        if pane_id:
            run_tmux('send-keys', '-t', pane_id, 'C-c', check=False)

    time.sleep(SHUTDOWN_WAIT_SECONDS)

    if session_exists():
        run_tmux('kill-session', '-t', f'={SESSION_NAME}', check=False)

    print('Ratgun stack stopped.')


def shell_command(workspace, setup_files, command):
    parts = [f'cd {shlex.quote(str(workspace))}']
    parts.extend(
        f'source {shlex.quote(str(setup_file))}'
        for setup_file in setup_files
    )
    parts.append(f'exec {command}')
    return ' && '.join(parts)


def add_window(window_name, command, first=False):
    tmux_command = [
        'new-session' if first else 'new-window',
        '-d' if first else '-t',
    ]

    if first:
        tmux_command.extend([
            '-s',
            SESSION_NAME,
            '-n',
            window_name,
            'bash',
            '-lc',
            command,
        ])
    else:
        tmux_command.extend([
            f'={SESSION_NAME}',
            '-n',
            window_name,
            'bash',
            '-lc',
            command,
        ])

    run_tmux(*tmux_command)


def validate_environment(workspace, camera_mode):
    required_files = [
        Path('/opt/ros/humble/setup.bash'),
        workspace / 'install' / 'setup.bash',
    ]

    if camera_mode:
        required_files.extend([
            Path('/opt/tros/humble/setup.bash'),
            workspace / 'install' / 'local_setup.bash',
        ])

    missing_files = [path for path in required_files if not path.is_file()]
    if missing_files:
        formatted = '\n'.join(f'  {path}' for path in missing_files)
        raise RuntimeError(f'Required setup files are missing:\n{formatted}')

    if shutil.which('tmux') is None:
        raise RuntimeError('tmux is not installed on this RDK system')

    if os.geteuid() != 0:
        raise RuntimeError(
            'Run this stack as root. Use "sudo -i" before starting it.'
        )


def camera_command(publish_pointcloud):
    arguments = [
        'ros2 launch hobot_stereonet',
        'stereonet_model_web_visual_v2.4.launch.py',
        'stereonet_pub_web:=False',
        f'publish_pcd_enabled:={str(publish_pointcloud)}',
        'publish_visual_enabled:=False',
        'publish_origin_enable:=False',
        'publish_rectify_bgr:=True',
        'mipi_image_width:=640',
        'mipi_image_height:=352',
        'mipi_sub_image_width:=640',
        'mipi_sub_image_height:=352',
        'mipi_image_framerate:=30.0',
        'mipi_gdc_enable:=True',
    ]

    if publish_pointcloud:
        arguments.extend([
            'pointcloud_downsample_step:=2',
            'pointcloud_depth_max:=4.0',
            'pcl_filter_enable:=False',
            'speckle_filter_enable:=False',
            'save_result_flag:=False',
        ])

    arguments.append('log_level:=warn')
    return ' '.join(arguments)


def start_stack(mode):
    workspace = Path(__file__).resolve().parent
    validate_environment(workspace, camera_mode=mode is not None)

    if session_exists():
        raise RuntimeError(
            f"The tmux session '{SESSION_NAME}' already exists. "
            'Run this script with --stop first.'
        )

    ros_setup = Path('/opt/ros/humble/setup.bash')
    tros_setup = Path('/opt/tros/humble/setup.bash')
    workspace_setup = workspace / 'install' / 'setup.bash'
    workspace_local_setup = workspace / 'install' / 'local_setup.bash'

    core_setup_files = [ros_setup, workspace_setup]
    target_setup_files = [tros_setup, workspace_local_setup]

    windows = [
        (
            'tf_tree',
            shell_command(
                workspace,
                core_setup_files,
                'ros2 launch tf_tree_pkg tf_tree.launch.py',
            ),
        ),
        (
            'closed_loop',
            shell_command(
                workspace,
                core_setup_files,
                'ros2 launch closed_loop_pkg closed_loop.launch.py',
            ),
        ),
        (
            'arduino',
            shell_command(
                workspace,
                core_setup_files,
                'ros2 run arduino_ros_bridge arduino_ros_bridge '
                f'--ros-args -p port_name:={ARDUINO_PORT}',
            ),
        ),
    ]

    if mode == 'april_tags':
        windows.extend([
            (
                'camera',
                shell_command(
                    workspace,
                    [tros_setup],
                    camera_command(publish_pointcloud=False),
                ),
            ),
            (
                'target',
                shell_command(
                    workspace,
                    target_setup_files,
                    'ros2 launch target_tf_pkg april_tags.launch.py',
                ),
            ),
        ])
    elif mode == 'red_point':
        windows.extend([
            (
                'camera',
                shell_command(
                    workspace,
                    [tros_setup],
                    camera_command(publish_pointcloud=True),
                ),
            ),
            (
                'target',
                shell_command(
                    workspace,
                    target_setup_files,
                    'ros2 launch target_tf_pkg '
                    'coordinate_publisher_ordered.launch.py',
                ),
            ),
        ])

    try:
        for index, (window_name, command) in enumerate(windows):
            add_window(window_name, command, first=index == 0)

        run_tmux(
            'set-option',
            '-t',
            f'={SESSION_NAME}',
            'remain-on-exit',
            'on',
        )
        run_tmux(
            'select-window',
            '-t',
            f'={SESSION_NAME}:tf_tree',
        )
    except Exception:
        if session_exists():
            run_tmux('kill-session', '-t', f'={SESSION_NAME}', check=False)
        raise

    print(f"Started tmux session '{SESSION_NAME}'.")

    if not sys.stdin.isatty() or not sys.stdout.isatty():
        print(f'Attach with: tmux attach-session -t {SESSION_NAME}')
        return

    os.execvp(
        'tmux',
        ['tmux', 'attach-session', '-t', f'={SESSION_NAME}'],
    )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Launch the Ratgun ROS 2 stack in tmux windows.'
    )
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument(
        '--april_tags',
        action='store_true',
        help='Use AprilTag target detection and the image-only camera mode.',
    )
    modes.add_argument(
        '--red_point',
        action='store_true',
        help='Use red-point detection and enable the point cloud.',
    )
    parser.add_argument(
        '--stop',
        action='store_true',
        help='Stop the running Ratgun tmux session cleanly.',
    )
    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.stop:
        if args.april_tags or args.red_point:
            raise RuntimeError('--stop cannot be combined with a target mode')
        stop_stack()
        return

    mode = None
    if args.april_tags:
        mode = 'april_tags'
    elif args.red_point:
        mode = 'red_point'

    start_stack(mode)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(f'run_stack.py: {error}', file=sys.stderr)
        sys.exit(1)
