import numpy as np


def reset_filter():
    for attr in ("initialized", "s", "M", "prev_x", "prev_y", "predicted_curve"):
        if hasattr(filter_coordinate, attr):
            delattr(filter_coordinate, attr)


def filter_coordinate(x, y):
    """
    Input:
        x, y : measured coordinates

    Return:
        tracked_state : [x, y, theta, v, omega]
        future_state  : [x, y, theta, v, omega] after l steps
    """

    dt = 0.1          # time step
    l = 30            # predict 10 steps into future

    # ---------- helper ----------
    def wrap_angle(a):
        return (a + np.pi) % (2 * np.pi) - np.pi

    # ---------- initialize once ----------
    if not hasattr(filter_coordinate, "initialized"):
        filter_coordinate.initialized = True

        filter_coordinate.s = np.array([
            x,      # x
            y,      # y
            0.0,    # theta unknown
            0.0,    # v unknown
            0.0     # omega unknown
        ], dtype=float)

        filter_coordinate.M = np.eye(5) * 1.0

        filter_coordinate.prev_x = x
        filter_coordinate.prev_y = y

        tracked_state = filter_coordinate.s.copy()
        future_state = tracked_state.copy()
        filter_coordinate.predicted_curve = [(tracked_state[0], tracked_state[1])]
        return tracked_state, future_state

    # ---------- get old state ----------
    s = filter_coordinate.s
    M = filter_coordinate.M

    px, py, th, v, om = s

    # ---------- noise matrices ----------
    Q = np.diag([
        0.01,   # x process noise
        0.01,   # y process noise
        0.05,   # theta process noise
        0.1,    # velocity process noise
        0.1     # omega process noise
    ])

    R = np.diag([
        0.05,   # x measurement noise
        0.05    # y measurement noise
    ])

    # ---------- EKF PREDICTION ----------
    s_pred = np.array([
        px + v * np.cos(th) * dt,
        py + v * np.sin(th) * dt,
        wrap_angle(th + om * dt),
        v,
        om
    ])

    # Jacobian A[n-1]
    A = np.array([
        [1, 0, -v * np.sin(th) * dt, np.cos(th) * dt, 0],
        [0, 1,  v * np.cos(th) * dt, np.sin(th) * dt, 0],
        [0, 0, 1,                    0,              dt],
        [0, 0, 0,                    1,              0],
        [0, 0, 0,                    0,              1]
    ])

    M_pred = A @ M @ A.T + Q

    # ---------- EKF UPDATE ----------
    z = np.array([x, y])

    # measurement model: we only observe x and y
    H = np.array([
        [1, 0, 0, 0, 0],
        [0, 1, 0, 0, 0]
    ])

    z_pred = H @ s_pred
    innovation = z - z_pred

    S = H @ M_pred @ H.T + R
    K = M_pred @ H.T @ np.linalg.inv(S)

    s_new = s_pred + K @ innovation
    s_new[2] = wrap_angle(s_new[2])

    M_new = (np.eye(5) - K @ H) @ M_pred

    # ---------- extra correction for theta from motion direction ----------
    dx = x - filter_coordinate.prev_x
    dy = y - filter_coordinate.prev_y

    if np.sqrt(dx**2 + dy**2) > 1e-4:
        measured_theta = np.arctan2(dy, dx)
        s_new[2] = wrap_angle(0.8 * s_new[2] + 0.2 * measured_theta)

        measured_v = np.sqrt(dx**2 + dy**2) / dt
        s_new[3] = 0.8 * s_new[3] + 0.2 * measured_v

    filter_coordinate.prev_x = x
    filter_coordinate.prev_y = y

    # save updated state
    filter_coordinate.s = s_new
    filter_coordinate.M = M_new

    tracked_state = s_new.copy()

    # ---------- FUTURE PREDICTION ----------
    future_state = tracked_state.copy()
    predicted_curve = [(tracked_state[0], tracked_state[1])]

    for _ in range(l):
        fx, fy, fth, fv, fom = future_state

        future_state = np.array([
            fx + fv * np.cos(fth) * dt,
            fy + fv * np.sin(fth) * dt,
            wrap_angle(fth + fom * dt),
            fv,
            fom
        ])
        predicted_curve.append((future_state[0], future_state[1]))

    filter_coordinate.predicted_curve = predicted_curve
    return tracked_state, future_state
