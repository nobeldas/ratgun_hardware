import numpy as np


def reset_filter():
    for attr in (
        "samples",
        "max_samples",
        "dt",
        "prediction_steps",
        "predicted_curve",
    ):
        if hasattr(filter_coordinate, attr):
            delattr(filter_coordinate, attr)


def _fit_line(values, dt):
    times = np.arange(values.size, dtype=float) * dt
    design = np.column_stack((times, np.ones(values.size, dtype=float)))
    slope, intercept = np.linalg.lstsq(design, values, rcond=None)[0]
    return slope, intercept


def filter_coordinate(x, y):
    if not hasattr(filter_coordinate, "samples"):
        filter_coordinate.samples = []
        filter_coordinate.max_samples = 12
        filter_coordinate.dt = 0.01
        filter_coordinate.prediction_steps = 10

    samples = filter_coordinate.samples
    samples.append((float(x), float(y)))
    if len(samples) > filter_coordinate.max_samples:
        del samples[:-filter_coordinate.max_samples]

    if len(samples) < 2:
        filter_coordinate.predicted_curve = [(float(x), float(y))]
        return float(x), float(y)

    sample_array = np.array(samples, dtype=float)
    x_slope, x_intercept = _fit_line(sample_array[:, 0], filter_coordinate.dt)
    y_slope, y_intercept = _fit_line(sample_array[:, 1], filter_coordinate.dt)

    start_index = len(samples) - 1
    predicted_curve = []
    for step in range(filter_coordinate.prediction_steps + 1):
        future_time = (start_index + step) * filter_coordinate.dt
        predicted_curve.append(
            (
                float(x_slope * future_time + x_intercept),
                float(y_slope * future_time + y_intercept),
            )
        )

    filter_coordinate.predicted_curve = predicted_curve
    return predicted_curve[-1]
