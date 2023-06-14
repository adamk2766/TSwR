import numpy as np
from observers.eso import ESO
from .controller import Controller
from numpy.linalg import inv
from models.manipulator_model import ManiuplatorModel

class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd



        # A = None
        A=np.array([[0,1,0],
                   [0,0,1],
                    [0,0,0]])
        # B = None
        B = np.array([[0],
                        [self.b],
                        [0]])


        # L = None
        L = np.array([[3 * (-p)],
                   [3 * ((-p) ** 2)],
                    [(-p) ** 3]])
        W = np.array([1, 0, 0])
        self.eso = ESO(A, B, W, L, q0, Tp)
        self.model = ManiuplatorModel(Tp)
    def set_b(self, b):
        ### TODO update self.b and B in ESO

        self.b = b

        B = np.array([[0],
                      [b],
                      [0]])

        self.eso.set_B(B)
        print('self.eso==============================================',self.eso)


        # return NotImplementedError

    def calculate_control(self,i, x2, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement ADRC

        q, q_dot = x
        print('x===',x)
        z=self.eso.get_state()
        print('z====',z)
        x_d=z[0]
        x_d_dot=z[1]
        f=z[2]



        inv_M = inv(self.model.M([0.0, x[0], 0.0, x[1]]))
        b=inv_M[i][i]
        self.set_b(b)

        v = q_d_ddot +self.kd * (x_d_dot - q_d_dot) + self.kp * (q-q_d)

        u=(v-f)/self.b
        self.eso.update(q,u)

        return u

        # return NotImplementedError
