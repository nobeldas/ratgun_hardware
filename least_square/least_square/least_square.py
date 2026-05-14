import numpy as np


def reset_filter():
    for attr in (
        "initialized",
        "dt",
        "prediction_steps",
        "state",
        "covariance",
        "transition",
        "measurement",
        "process_noise",
        "measurement_noise",
        "predicted_curve",
    ):
        if hasattr(filter_coordinate, attr):
            delattr(filter_coordinate, attr)


def filter_coordinate(x, y):
    if not hasattr(filter_coordinate, "initialized"):
        dt = 0.01
        filter_coordinate.dt = dt
        filter_coordinate.prediction_steps = 10
        filter_coordinate.state = np.array([x, y, 0.0, 0.0], dtype=float)
        filter_coordinate.covariance = np.eye(4, dtype=float) * 500.0
        filter_coordinate.transition = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        filter_coordinate.measurement = np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
            ],
            dtype=float,
        )
        filter_coordinate.process_noise = np.diag([0.25, 0.25, 100.0, 100.0])
        filter_coordinate.measurement_noise = np.diag([1.0, 1.0])
        filter_coordinate.predicted_curve = [(float(x), float(y))]
        filter_coordinate.initialized = True
        return float(x), float(y)

    state = filter_coordinate.state
    covariance = filter_coordinate.covariance
    transition = filter_coordinate.transition
    measurement = filter_coordinate.measurement
    process_noise = filter_coordinate.process_noise
    measurement_noise = filter_coordinate.measurement_noise

    predicted_state = transition @ state
    predicted_covariance = transition @ covariance @ transition.T + process_noise

    observed = np.array([x, y], dtype=float)
    innovation = observed - measurement @ predicted_state
    innovation_covariance = (
        measurement @ predicted_covariance @ measurement.T + measurement_noise
    )
    kalman_gain = (
        predicted_covariance
        @ measurement.T
        @ np.linalg.inv(innovation_covariance)
    )

    state = predicted_state + kalman_gain @ innovation
    covariance = (np.eye(4) - kalman_gain @ measurement) @ predicted_covariance

    future_state = state.copy()
    predicted_curve = [(float(future_state[0]), float(future_state[1]))]
    for _ in range(filter_coordinate.prediction_steps):
        future_state = transition @ future_state
        predicted_curve.append((float(future_state[0]), float(future_state[1])))

    filter_coordinate.state = state
    filter_coordinate.covariance = covariance
    filter_coordinate.predicted_curve = predicted_curve
    return float(future_state[0]), float(future_state[1])
