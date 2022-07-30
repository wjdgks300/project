import numpy as np
import matplotlib.pyplot as plt


def kalman_xy(x, P, measurement, R, motion, Q):
    """
    Parameters:
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot) x_dot = 속도
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
    return kalman(x, P, measurement, R, motion, Q,
                  # x = x + 1*x_dot  -> x_dot 속도를 통해 1초 후의 어떠한 물페 x 의 위치를 구하는 식
                  F=np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      '''),
                  H=np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))
                # H는 측정한 x,y값을 빼내는 함수

def kalman(x, P, measurement, R, motion, Q, F, H):
    '''
    Parameters:
    x: initial state
    P: initial uncertainty convariance matrix                   공분산 과정
    measurement: observed position (same shape as H*x)
    R: measurement noise (same shape as H)
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    F: next state function: x_prime = F*x                운동 방적식과 같은 기존의 알려진 물리 방정식
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
    # kalman gain은 새 추정치를 업데이트 하는 데 사용할 새 측정값의 양을 결정하는 데 사용됨

    x = x + K * y                       # 사후 예측치
    I = np.matrix(np.eye(F.shape[0]))  # identity matrix
    P = (I - K * H) * P                 # 사후 공분산

    # PREDICT x, P based on motion
    x_hat = F * x + motion                # 사전 예상한 값
    P_hat = F * P * F.T + Q               # 사전 공분산

    # print(K)
    # print(P)
    # print(Q)

    return x_hat, P_hat, x


def demo_kalman_xy():
    x = np.matrix('0. 0. 0. 0.').T
    P = np.matrix(np.eye(4)) * 1000  # initial uncertainty
    # 두 벡터의 불확실성에 대한 행렬 -> 처음실해할 때 불확실성 높게 1000으로 잡아줌
    # print(P)

    N = 20
    true_x = np.linspace(0.0, 10.0, N)
    true_y = true_x ** 2
    scale = 0.5
    sigma_q = 0.1
    sigma_r = 0.1
    observed_x = true_x + scale * np.random.random(N)
    observed_y = true_y + scale * np.random.random(N)
    plt.plot(observed_x, observed_y, 'ro')                  # 측정된 x, y 값 빨간점
    result = []
    _result = []
    motion = np.matrix('0. 0. 0. 0.').T
    Q = np.matrix(np.eye(4)) * (sigma_q ** 2)
    R = sigma_r ** 2
    for meas in zip(observed_x, observed_y):
        x, P, _x = kalman_xy(x, P, meas, R, motion, Q)
        result.append((x[:2]).tolist())
        _result.append((_x[:2]).tolist())
        print(P)
    x, y = zip(*result)
    _x, _y = zip(*_result)
    plt.plot(x, y, 'go')                               # 초록 사전 예측치
    plt.plot(_x, _y, 'b-')
    plt.show()


demo_kalman_xy()
