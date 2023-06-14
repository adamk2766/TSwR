import numpy as np
from observers.eso import ESO
from .controller import Controller


class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd

        A = np.array([[0, 1, 0],
                      [0, 0, 1],
                      [0, 0, 0]])
        B = np.array([[0],
                      [b],
                      [0]])
        L = np.array([[3*p],
                      [3*p**2],
                      [p**3]])
        W = np.array([[1, 0, 0]])
        self.eso = ESO(A, B, W, L, q0, Tp)

    def set_b(self, b):
        self.b = b
        B = np.array([[0],
                      [b],
                      [0]])
        self.eso.set_B(B)

    def calculate_control(self, i, x, q_d, q_d_dot, q_d_ddot):
        q = x[0]
        z_hat = self.eso.get_state()
        x_hat = z_hat[0]
        x_hat_dot = z_hat[1]
        f = z_hat[2]
        e = q_d - q
        e_dot = q_d_dot - x_hat_dot
        v = q_d_ddot + self.kd * e_dot + self.kp * e
        u = (v - f) / self.b
        self.eso.update(q, u)

        l1 = 0.5
        r1 = 0.01
        m1 = 1.
        l2 = 0.5
        r2 = 0.01
        m2 = 1.
        I_1 = 1 / 12 * m1 * (3 * r1 ** 2 + l1 ** 2)
        I_2 = 1 / 12 * m2 * (3 * r2 ** 2 + l2 ** 2)
        m3 = 0.0
        r3 = 0.01
        I_3 = 2. / 5 * m3 * r3 ** 2
        M = np.zeros((2, 2))
        d1 = l1 / 2
        d2 = l2 / 2
        d1_1 = d1 * d1
        d2_2 = d2 * d2
        M[0, 0] = m1 * d1_1 + I_1 + m2 * l1 ** 2 + m2 * d2_2 + 2 * m2 * l1 * d2 * np.cos(x_hat) + I_2 + m3 * l1 ** 2 + m3 * l2 ** 2 + 2 * m3 * l1 * l2 * np.cos(x_hat) + I_3
        M[0, 1] = m2 * d2_2 + I_2 + m3 * l2 ** 2 + m2 * l1 * d2 * np.cos(x_hat) + m3 * l1 * l2 * np.cos(x_hat) + I_3
        M[1, 0] = m2 * d2_2 + m2 * l1 * d2 * np.cos(x_hat) + I_2 + m3 * l2 ** 2 + m3 * l1 * l2 * np.cos(x_hat) + I_3
        M[1, 1] = m2 * d2_2 + I_2 + m3 * l2 ** 2 + I_3
        M_inv = np.linalg.inv(M)
        self.set_b(M_inv[i, i])
        return u
