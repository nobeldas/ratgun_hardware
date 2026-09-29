# Ratgun Hardware Stack

ROS 2 workspace and startup tooling for the Ratgun hardware running on the
RDK100.

## Connect to the RDK100

```bash
sshpass -p adminisadmin ssh sunrise@192.168.0.107
```

The stack must be started as root:

```bash
sudo -i
cd /home/sunrise/ratgun_hardware
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Start the stack

Core stack only:

```bash
./run_stack.py
```

AprilTag target detection:

```bash
./run_stack.py --april_tags
```

Red-point target detection:

```bash
./run_stack.py --red_point
```

Green-point target detection:

```bash
./run_stack.py --green_point
```

Use projectile-compensated pan and tilt instead of the normal pan/tilt node:

```bash
./run_stack.py --red_point --proj
```

## Target prediction

The prediction options can be combined with `--april_tags`, `--red_point`,
`--green_point`, and `--proj`. Selecting a predictor also starts the flight-time
node that publishes `/seperate_tof`.

Start CRLB least-squares prediction:

```bash
./run_stack.py --red_point --crlb
```

The CRLB node publishes:

- Point: `/predicted_target_position`
- TF: `base_link -> target_prediction_tf`

Start constant-velocity Kalman prediction:

```bash
./run_stack.py --red_point --kalman
```

The Kalman node publishes:

- Point: `/kalman_predicted_target_position`
- TF: `base_link -> target_kalman_prediction_tf`

Example using AprilTags, projectile compensation, and Kalman prediction:

```bash
./run_stack.py --april_tags --proj --kalman
```

Example using green-point detection, projectile compensation, and Kalman
prediction:

```bash
./run_stack.py --green_point --proj --kalman
```

`--crlb` and `--kalman` are mutually exclusive. If neither is supplied, the
stack runs without a target-prediction node.

Both predictors wait for a valid `/seperate_tof` message and do not use a
default flight time. Their output timestamp is the latest target measurement
timestamp plus the received flight time.

## Controlled shutdown

From the workspace directory, run:

```bash
./run_stack.py --stop
```

This sends `Ctrl-C` to every process in the `ratgun` tmux session, waits five
seconds for ROS nodes to shut down cleanly, and then removes the tmux session if
it is still running.
