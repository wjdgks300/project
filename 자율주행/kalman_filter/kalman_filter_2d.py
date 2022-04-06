import numpy as np
import matplotlib.pyplot as plt


def kalman_xy(x, P, measurement, R, motion, Q):
    """
    Parameters:
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
    return kalman(x, P, measurement, R, motion, Q,
                  F=np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      '''),
                  H=np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))


def kalman(x, P, measurement, R, motion, Q, F, H):
    '''
    Parameters:
    x: initial state
    P: initial uncertainty convariance matrix
    measurement: observed position (same shape as H*x)
    R: measurement noise (same shape as H)
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    F: next state function: x_prime = F*x
    H: measurement function: position = H*x

    Return: the updated and predicted new values for (x, P)

    See also http://en.wikipedia.org/wiki/Kalman_filter

    This version of kalman can be applied to many different situations by
    appropriately defining F and H
    '''
    # UPDATE x, P based on measurement m
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R  # residual convariance
    K = P * H.T * S.I  # Kalman gain
    x = x + K * y
    I = np.matrix(np.eye(F.shape[0]))  # identity matrix
    P = (I - K * H) * P

    # PREDICT x, P based on motion
    x_hat = F * x + motion
    P_hat = F * P * F.T + Q

    # print(K)
    # print(P)
    # print(Q)

    return x_hat, P_hat, x


def demo_kalman_xy():
    x = np.matrix('0. 0. 0. 0.').T
    P = np.matrix(np.eye(4)) * 1000  # initial uncertainty
    # print(P)

    N = 20
    true_x = np.linspace(0.0, 10.0, N)
    true_y = true_x ** 2
    scale = 0.5
    sigma_q = 0.1
    sigma_r = 0.1
    observed_x = true_x + scale * np.random.random(N)
    observed_y = true_y + scale * np.random.random(N)
    plt.plot(observed_x, observed_y, 'ro')
    result = []
    _result = []
    motion = np.matrix('0. 0. 0. 0.').T
    Q = np.matrix(np.eye(4)) * (sigma_q ** 2)
    R = sigma_r ** 2
    for meas in zip(observed_x, observed_y):
        x, P, _x = kalman_xy(x, P, meas, R, motion, Q)
        result.append((x[:2]).tolist())
        _result.append((_x[:2]).tolist())
        # print(P)
    x, y = zip(*result)
    _x, _y = zip(*_result)
    plt.plot(x, y, 'go')
    plt.plot(_x, _y, 'b-')
    plt.show()


demo_kalman_xy()