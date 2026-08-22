import numpy as np


# -------------------- UKF tuning knobs --------------------
# Increase DT if your incoming measurements are slower than the simulator loop.
# Decrease DT if the filter overshoots because updates arrive very quickly.
DT = 0.05

# Increase PREDICTION_STEPS to draw/publish farther into the future.
# Decrease it if the magenta future path looks too long or unreliable.
PREDICTION_STEPS = 30

# UKF sigma-point spread. ALPHA=1.0 keeps nonnegative mean weights here, which
# avoids angle-wrap flips during future prediction. Try 0.5-0.8 only if the
# filter is too conservative and you can tolerate more prediction noise.
ALPHA = 1.0
BETA = 2.0
KAPPA = 0.0

# Initial uncertainty for [x, y, theta, velocity, omega].
# Increase velocity/omega if startup motion takes too long to lock on.
# Increase theta only if the first predicted curve refuses to turn correctly.
INITIAL_COVARIANCE = np.diag([1.0, 1.0, 0.1, 1.0, 1.0])

# Process noise for [x, y, theta, velocity, omega].
# Increase velocity/omega noise when the robot speed changes suddenly.
# Decrease them for a smoother but slower-to-react estimate.
PROCESS_NOISE = np.diag([0.01, 0.01, 0.05, 0.1, 0.1])

# Scale process noise during the no-measurement future rollout.
# Increase toward 1.0 to show more uncertainty-driven curve spreading.
# Keep near 0.0-0.2 when you want a stable best-guess future path.
FUTURE_PROCESS_NOISE_SCALE = 0.0

# Scale the current covariance before projecting the future curve.
# Use 0.0 for a clean best-guess path. Use 0.2-1.0 if you want the future
# path to reflect more uncertainty from noisy measurements.
FUTURE_COVARIANCE_SCALE = 0.0

# Measurement noise for direct x/y observations.
# Increase these when the orange measured path is noisy and the output jitters.
# Decrease them when measurements are clean and the green output lags too much.
POSITION_MEASUREMENT_NOISE = np.diag([0.05, 0.05])

# Extra pseudo-measurement noise for heading and speed estimated from x/y deltas.
# Increase these if heading flips or speed spikes during noisy/slow movement.
# Decrease them if future prediction does not turn or accelerate quickly enough.
HEADING_SPEED_MEASUREMENT_NOISE = np.diag([0.3, 0.5])

# Ignore tiny x/y deltas when estimating heading from consecutive measurements.
# Increase it if the robot is almost stationary and heading jitters.
MIN_HEADING_DISTANCE = 1e-4


def reset_filter():
    for attr in (
        "initialized",
        "state",
        "covariance",
        "prev_x",
        "prev_y",
        "predicted_curve",
    ):
        if hasattr(filter_coordinate, attr):
            delattr(filter_coordinate, attr)


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def motion_model(state, dt):
    x, y, theta, velocity, omega = state
    return np.array(
        [
            x + velocity * np.cos(theta) * dt,
            y + velocity * np.sin(theta) * dt,
            wrap_angle(theta + omega * dt),
            velocity,
            omega,
        ],
        dtype=float,
    )


def sigma_points(mean, covariance, alpha=ALPHA, beta=BETA, kappa=KAPPA):
    n = mean.size
    lambda_ = alpha**2 * (n + kappa) - n
    scale = n + lambda_

    jitter = 1e-9
    while True:
        try:
            sqrt_covariance = np.linalg.cholesky(scale * covariance)
            break
        except np.linalg.LinAlgError:
            covariance = covariance + np.eye(n) * jitter
            jitter *= 10.0

    points = [mean]
    for i in range(n):
        points.append(mean + sqrt_covariance[:, i])
        points.append(mean - sqrt_covariance[:, i])

    wm = np.full(2 * n + 1, 1.0 / (2.0 * scale))
    wc = wm.copy()
    wm[0] = lambda_ / scale
    wc[0] = lambda_ / scale + (1.0 - alpha**2 + beta)
    return np.array(points, dtype=float), wm, wc


def state_mean(points, weights):
    mean = np.zeros(points.shape[1], dtype=float)
    mean[0] = np.sum(weights * points[:, 0])
    mean[1] = np.sum(weights * points[:, 1])
    mean[2] = np.arctan2(
        np.sum(weights * np.sin(points[:, 2])),
        np.sum(weights * np.cos(points[:, 2])),
    )
    mean[3] = np.sum(weights * points[:, 3])
    mean[4] = np.sum(weights * points[:, 4])
    return mean


def state_residual(value, mean):
    residual = value - mean
    residual[2] = wrap_angle(residual[2])
    return residual


def measurement_mean(points, weights, has_heading):
    mean = np.zeros(points.shape[1], dtype=float)
    mean[0] = np.sum(weights * points[:, 0])
    mean[1] = np.sum(weights * points[:, 1])
    if has_heading:
        mean[2] = np.arctan2(
            np.sum(weights * np.sin(points[:, 2])),
            np.sum(weights * np.cos(points[:, 2])),
        )
        mean[3] = np.sum(weights * points[:, 3])
    return mean


def measurement_residual(value, mean, has_heading):
    residual = value - mean
    if has_heading:
        residual[2] = wrap_angle(residual[2])
    return residual


def predict_distribution(state, covariance, dt, process_noise_scale=1.0):
    points, wm, wc = sigma_points(state, covariance)
    predicted_points = np.array([motion_model(point, dt) for point in points])
    predicted_state = state_mean(predicted_points, wm)

    predicted_covariance = PROCESS_NOISE * process_noise_scale
    for weight, point in zip(wc, predicted_points):
        residual = state_residual(point, predicted_state)
        predicted_covariance += weight * np.outer(residual, residual)

    return predicted_state, predicted_covariance, predicted_points, wm, wc


def predict_future_curve(state, covariance):
    future_state = state.copy()
    future_covariance = covariance * FUTURE_COVARIANCE_SCALE
    predicted_curve = [(float(future_state[0]), float(future_state[1]))]

    # Future prediction is UKF-only: repeatedly propagate sigma points through
    # the diff-drive motion model without applying new measurements.
    for _ in range(PREDICTION_STEPS):
        future_state, future_covariance, _, _, _ = predict_distribution(
            future_state, future_covariance, DT, FUTURE_PROCESS_NOISE_SCALE
        )
        predicted_curve.append((float(future_state[0]), float(future_state[1])))

    return future_state, predicted_curve


def filter_coordinate(x, y):
    """
    Unscented Kalman Filter for a diff-drive state:
        [x, y, theta, velocity, omega]

    Input:
        x, y : measured coordinates

    Return:
        tracked_state : current UKF state estimate
        future_state  : state after prediction_steps future steps
    """

    if not hasattr(filter_coordinate, "initialized"):
        filter_coordinate.initialized = True
        filter_coordinate.state = np.array([x, y, 0.0, 0.0, 0.0], dtype=float)
        filter_coordinate.covariance = INITIAL_COVARIANCE.copy()
        filter_coordinate.prev_x = x
        filter_coordinate.prev_y = y

        tracked_state = filter_coordinate.state.copy()
        future_state = tracked_state.copy()
        filter_coordinate.predicted_curve = [(tracked_state[0], tracked_state[1])]
        return tracked_state, future_state

    state = filter_coordinate.state
    covariance = filter_coordinate.covariance

    predicted_state, predicted_covariance, predicted_points, wm, wc = (
        predict_distribution(state, covariance, DT)
    )

    dx = x - filter_coordinate.prev_x
    dy = y - filter_coordinate.prev_y
    distance = np.hypot(dx, dy)
    has_heading = distance > MIN_HEADING_DISTANCE

    if has_heading:
        observed = np.array(
            [x, y, np.arctan2(dy, dx), distance / DT],
            dtype=float,
        )
        measurement_noise = np.block(
            [
                [POSITION_MEASUREMENT_NOISE, np.zeros((2, 2))],
                [np.zeros((2, 2)), HEADING_SPEED_MEASUREMENT_NOISE],
            ]
        )
        measurement_points = predicted_points[:, [0, 1, 2, 3]]
    else:
        observed = np.array([x, y], dtype=float)
        measurement_noise = POSITION_MEASUREMENT_NOISE.copy()
        measurement_points = predicted_points[:, [0, 1]]

    predicted_observation = measurement_mean(measurement_points, wm, has_heading)

    innovation_covariance = measurement_noise.copy()
    cross_covariance = np.zeros((state.size, observed.size), dtype=float)
    for weight, state_point, measurement_point in zip(
        wc, predicted_points, measurement_points
    ):
        state_error = state_residual(state_point, predicted_state)
        measurement_error = measurement_residual(
            measurement_point, predicted_observation, has_heading
        )
        innovation_covariance += weight * np.outer(
            measurement_error, measurement_error
        )
        cross_covariance += weight * np.outer(state_error, measurement_error)

    kalman_gain = cross_covariance @ np.linalg.inv(innovation_covariance)
    innovation = measurement_residual(observed, predicted_observation, has_heading)

    tracked_state = predicted_state + kalman_gain @ innovation
    tracked_state[2] = wrap_angle(tracked_state[2])
    covariance = predicted_covariance - kalman_gain @ innovation_covariance @ kalman_gain.T
    covariance = 0.5 * (covariance + covariance.T)

    filter_coordinate.prev_x = x
    filter_coordinate.prev_y = y
    filter_coordinate.state = tracked_state
    filter_coordinate.covariance = covariance

    future_state, predicted_curve = predict_future_curve(tracked_state, covariance)
    filter_coordinate.predicted_curve = predicted_curve
    return tracked_state.copy(), future_state.copy()
