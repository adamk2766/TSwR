import numpy as np


class ManiuplatorModel:
    # , m3, r3
    def __init__(self, Tp):

        self.Tp = Tp
        self.l1 = 0.5
        self.r1 = 0.04
        self.m1 = 1.
        self.l2 = 0.4
        self.r2 = 0.04
        self.m2 = 2.4
        self.I_1 = 1 / 12 * self.m1 * (3 * self.r1 ** 2 + self.l1 ** 2)
        self.I_2 = 1 / 12 * self.m2 * (3 * self.r2 ** 2 + self.l2 ** 2)
        self.m3 = 0.8
        self.r3 = 0.05
        self.I_3 = 2. / 5 * self.m3 * self.r3 ** 2


        self.d1 = self.l1 / 2
        self.d2 = self.l2 / 2

        self.alpha = self.m1 * self.d1 ** 2 + self.I_1 + self.m2 * (self.l1 ** 2 + self.d2 ** 2) + self.I_2 + self.m3 * (self.l1 ** 2 + self.l2 ** 2) + self.I_3
        self.beta = self.m2 * self.l1 * self.d2 + self.m3 * self.l1 * self.l2
        self.theta = self.m2 * self.d2 ** 2 + self.I_2 + self.m3 * self.l2 ** 2 + self.I_3

    def M(self, x):
        """
        Please implement the calculation of the mass matrix, according to the model derived in the exercise
        (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x


        Mass_m = np.array([[self.alpha+2*self.beta*np.cos(q2) , self.theta + self.beta*np.cos(q2)],
                    [self.theta+self.beta*np.cos(q2) , self.theta]])


        return Mass_m

    def C(self, x):
        """
        Please implement the calculation of the Coriolis and centrifugal forces matrix, according to the model derived
        in the exercise (2DoF planar manipulator with the object at the tip)
        """
        q1, q2, q1_dot, q2_dot = x



        Coriolis_m = np.array([[-self.beta * np.sin(q2 ) * q2_dot, -self.beta * np.sin(q2 ) * (q1_dot + q2_dot)],
             [self.beta * np.sin(q2) * q2_dot, 0]])




        return Coriolis_m

    def x_dot(self, x, u):
        invM = np.linalg.inv(self.M(x))
        zeros = np.zeros((2, 2), dtype=np.float32)
        # A = -inv(M)*C
        A = np.concatenate([np.concatenate([zeros, np.eye(2)], 1), np.concatenate([zeros, -invM @ self.C(x)], 1)], 0)
        # b = inv(M)
        b = np.concatenate([zeros, invM], 0)
        return A @ x[:, np.newaxis] + b @ u