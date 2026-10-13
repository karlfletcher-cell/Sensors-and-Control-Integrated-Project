
import numpy as np
from config import DT


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def wrap(angle):
    '''Wrap an angle to the range (-pi, pi].

    sin and cos turn the angle into a point on the unit circle,
    and arctan2 reads that point back as an angle in (-pi, pi].
    '''
    return np.arctan2(np.sin(angle), np.cos(angle))


def odometry_to_input(dl, dr, wheel_sep):
    '''Convert left/right wheel distances travelled in one step into u = [v, w].'''
    v = (dl + dr) / 2 / DT           # average wheel speed
    w = (dr - dl) / wheel_sep / DT   # turning rate
    return np.array([v, w])


# ---------------------------------------------------------------------------
# Motion model:  x_next = f(x, u)
# ---------------------------------------------------------------------------

def motion_model(xh, u):
    '''Predict the next pose from the current pose and the input.'''
    x, y, th = xh
    v, w = u

    if abs(w) < 1e-6:
        # Driving straight
        x_new = x + v * DT * np.cos(th)
        y_new = y + v * DT * np.sin(th)
        th_new = th
    else:
        # Driving along an arc
        th_new = th + w * DT
        x_new = x + (v / w) * (np.sin(th_new) - np.sin(th))
        y_new = y + (v / w) * (np.cos(th) - np.cos(th_new))

    return np.array([x_new, y_new, wrap(th_new)])


def motion_jacobian(xh, u):
    '''F = df/dx: how the next pose changes with the current pose.'''
    th = xh[2]
    v, w = u

    F = np.eye(3)
    if abs(w) < 1e-6:
        # Driving straight
        F[0, 2] = -v * DT * np.sin(th)
        F[1, 2] =  v * DT * np.cos(th)
    else:
        # Driving along an arc
        th_new = th + w * DT
        F[0, 2] = (v / w) * (np.cos(th_new) - np.cos(th))
        F[1, 2] = (v / w) * (np.sin(th_new) - np.sin(th))
    return F


# ---------------------------------------------------------------------------
# Measurement model:  z = h(x)
# ---------------------------------------------------------------------------

def predict_range_bearing(xh, mk):
    '''Expected range and bearing from pose xh to marker mk = [mx, my].'''
    dx = mk[0] - xh[0]
    dy = mk[1] - xh[1]

    rng = np.hypot(dx, dy)
    bearing = wrap(np.arctan2(dy, dx) - xh[2])
    return np.array([rng, bearing])


def measurement_jacobian(xh, mk):
    '''H = dh/dx: how range and bearing change with the pose.'''
    dx = mk[0] - xh[0]
    dy = mk[1] - xh[1]
    q = dx**2 + dy**2      # squared distance
    r = np.sqrt(q)         # distance

    return np.array([
        [-dx / r, -dy / r,  0.0],   # d(range)   / d(x, y, theta)
        [ dy / q, -dx / q, -1.0],   # d(bearing) / d(x, y, theta)
    ])


def range_bearing_residual(z, z_hat):
    '''Innovation z - z_hat, with the bearing difference wrapped.'''
    return np.array([z[0] - z_hat[0],
                     wrap(z[1] - z_hat[1])])


# ---------------------------------------------------------------------------
# EKF steps
# ---------------------------------------------------------------------------

def ekf_predict(xh, P, u, f, F_jac, Qd):
    '''
    Predict step.
        x_pred = f(x, u)
        P_pred = F P F^T + Q
    '''
    F = F_jac(xh, u)            # Jacobian at the current estimate
    xh_pred = f(xh, u)
    P_pred = F @ P @ F.T + Qd
    return xh_pred, P_pred


def ekf_update(xh_pred, P_pred, z, h, H_jac, Rn, residual=None):
    '''
    Update step.
        nu = z - h(x_pred)           innovation
        S  = H P H^T + R             innovation covariance
        K  = P H^T S^-1              Kalman gain
        x  = x_pred + K nu
        P  = (I - K H) P_pred

    residual: optional function for nu (e.g. range_bearing_residual,
              so the bearing difference is wrapped). Defaults to z - z_hat.
    '''
    z_hat = h(xh_pred)          # expected measurement
    H = H_jac(xh_pred)          # Jacobian at the predicted estimate

    if residual is None:
        nu = z - z_hat
    else:
        nu = residual(z, z_hat)

    S = H @ P_pred @ H.T + Rn
    K = P_pred @ H.T @ np.linalg.inv(S)

    xh = xh_pred + K @ nu
    xh[2] = wrap(xh[2])
    P = (np.eye(len(xh)) - K @ H) @ P_pred
    return xh, P, nu, S
