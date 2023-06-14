import numpy as np

# from models.free_model import FreeModel
from observers.eso import ESO
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
# from models.ideal_model import IdealModel
from numpy.linalg import inv
from models.manipulator_model import ManiuplatorModel

class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):


        self.model = ManiuplatorModel(Tp)
        self.Kp = Kp
        self.Kd = Kd

        p1, p2=p
        print(p)
        self.L = np.array([[3*(-p1), 0],
                           [0, 3*(-p2)],
                           [3*(-p1)**2, 0],
                           [0, 3*(-p2)**2],
                           [(-p1)**3, 0],
                           [0, (-p2)**3]])

        W = np.array([[1, 0, 0, 0, 0, 0],
                     [0, 1, 0, 0, 0, 0]])
        print('W',W )
        A = np.array([[0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 1],
                     [0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0]])

        # A[2:4, 2:4] = inv_M @ C

        B =  np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])
        # B[2:4, 0:2]=inv_M
        print(B)


        self.eso = ESO(A, B, W, self.L, q0, Tp)
        self.update_params(q0[:2], q0[2:])

    def update_params(self, q, q_dot):

        q1, q2,  = q
        q1_dot, q2_dot = q_dot

        inv_M = inv(self.model.M([q1, q2, q1_dot, q2_dot]))

        A = np.array([[0, 0, 1, 0, 0, 0],
                     [0, 0, 0, 1, 0, 0],
                     [0, 0, 0, 0, 1, 0],
                     [0, 0, 0, 0, 0, 1],
                     [0, 0, 0, 0, 0, 0],
                     [0, 0, 0, 0, 0, 0]])

        A[2:4, 2:4] = -inv_M @ self.model.C([q1, q2, q1_dot, q2_dot])

        B =  np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])
        B[2:4, :]=inv_M

        ### TODO Implement procedure to set eso.A and eso.B
        self.eso.A = A
        self.eso.B = B

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        print('type================================',type(q_d_ddot))

        z = self.eso.get_state()

        x_d=z[0:2]
        x_d_dot=z[2:4]

        f=z[4:6]
        q = x[0:2]
        e=np.array([q - q_d]).T
        e_dot = np.array([ q_d_dot - x_d_dot]).T
        # print('q_d',q_d_dot)
        v = np.array([q_d_ddot]).T + self.Kd @ e_dot + self.Kp @ e


        q1, q2, = x_d
        q1_dot, q2_dot = x_d_dot


        u = self.model.M(z[0:4]) @ (v - np.array([f]).T) + np.array([self.model.C(z[0:4]) @ x_d_dot]).T


        self.update_params(x_d, x_d_dot)
        self.eso.update(q, u)


        print('v',np.array([q_d_ddot]).T )
        ### TODO implement centralized ADRFLC
        return u

