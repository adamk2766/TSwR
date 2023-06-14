import numpy as np

# from models.free_model import FreeModel
from observers.eso import ESO
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
# from models.ideal_model import IdealModel
from models.manipulator_model import ManiuplatorModel

class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.model = ManiuplatorModel(Tp)
        self.Kp = Kp
        self.Kd = Kd
        p1 = p[0]
        p2 = p[1]
        self.L = np.array([[3*p1, 0],
                           [0, 3*p2],
                           [3*p1**2, 0],
                           [0, 3*p2**2],
                           [p1**3, 0],
                           [0, p2**3]])
        W = np.zeros((2, 6))
        W[0, 0] = 1
        W[1, 1] = 1
        A = np.zeros((6, 6))
        A[0, 2] = 1
        A[1, 3] = 1
        A[2, 4] = 1
        A[3, 5] = 1
        B = np.zeros((6, 2))
        self.eso = ESO(A, B, W, self.L, q0, Tp)
        self.update_params(q0[:2], q0[2:])

    def update_params(self, q, q_dot):
        A = np.zeros((6, 6))
        A[0, 2] = 1
        A[1, 3] = 1
        A[2, 4] = 1
        A[3, 5] = 1
        x = np.concatenate([q, q_dot], axis=0)
        M = self.model.M(x)
        M_inv = np.linalg.inv(M)
        C = self.model.C(x)
        M_in = -(M_inv @ C)
        A[2, 2] = M_in[0, 0]
        A[2, 3] = M_in[0, 1]
        A[3, 2] = M_in[1, 0]
        A[3, 3] = M_in[1, 1]
        self.eso.A = A

        B = np.zeros((6, 2))
        B[2, 0] = M_inv[0, 0]
        B[2, 1] = M_inv[0, 1]
        B[3, 0] = M_inv[1, 0]
        B[3, 1] = M_inv[1, 1]
        self.eso.B = B

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        q1, q2, q1_dot, q2_dot = x
        q = np.array([q1, q2])
        M = self.model.M(x)
        C = self.model.C(x)
        z_hat = self.eso.get_state()
        x_hat = z_hat[0:2]
        x_hat_dot = z_hat[2:4]
        f = z_hat[4:]
        e = q_d - q
        e_dot = q_d_dot - x_hat_dot
        v = q_d_ddot + self.Kd @ e_dot + self.Kp @ e
        u = M @ (v - f) + C @ x_hat_dot
        self.update_params(x_hat, x_hat_dot)
        self.eso.update(q.reshape(len(q), 1), u.reshape(len(u), 1))
        return u
