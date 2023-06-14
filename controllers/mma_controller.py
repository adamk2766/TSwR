import numpy as np
from .controller import Controller
from numpy.linalg import inv
from models.manipulator_model import ManiuplatorModel




class MMAController(Controller):
    def __init__(self, Tp):

        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        self.Tp=Tp
        # self.m3_1=0.1
        # self.r3_1=0.05
        # self.m3_2=0.8
        # self.r3_2=0.05
        # self.m3_3=1.0
        # self.r3_3=0.3


        model1=ManiuplatorModel(Tp)
        model1.m3=0.1
        model1.r3 = 0.05

        model2 = ManiuplatorModel(Tp)
        model2.m3=0.8
        model2.r3 = 0.05

        model3 = ManiuplatorModel(Tp)
        model3.m3=1.0
        model3.r3 = 0.3

        self.models = [model1,
                        model2,
                        model3]


        self.i = 0
        self.u_prev=np.array([[0],[0]])
        self.x_prev=np.array([0,0,0,0])
        self.x_compare = [0,0,0]
        self.x_c_dot_prev = [0,0,0]
    def choose_model(self, x):
        tab=[]
        x_err=[0,0,0]

        for i in range(0,3):
            inv_M = inv( self.models[i].M(self.x_prev) )
            invM_C = -inv_M @ self.models[i].C(self.x_prev)

            # print('invM_C',invM_C)

            A_q=np.array([[0,0,1,0],
                       [0,0,0,1],
                       [0,0,invM_C[0][0],invM_C[0][1]],
                       [0,0,invM_C[1][0],invM_C[1][1]]])
            B_q=np.array([[0,0],
                       [0,0],
                       [inv_M[0][0],inv_M[0][1]],
                       [inv_M[1][0],inv_M[1][1]]])



            x_c_dot = A_q @ self.x_prev[:, np.newaxis] + B_q @ self.u_prev
            # print('x_c_dot', x_c_dot)
            # print('model.x_dot(self.x_prev, self.u_prev)', self.models[i].x_dot(self.x_prev, self.u_prev))


            # print('invM_C', i)



            self.x_compare[i] = self.x_prev[0] + ((x_c_dot[2]+self.x_c_dot_prev[i])/2* self.Tp)
            print('self.x_compare[i]',i,':',self.x_compare[i])
            # print('x_err', i, ':', x_err[i])


            # print('self.x_prev',i,':',self.x_prev)


            self.x_c_dot_prev[i] = x_c_dot[2]

            tab.append(abs(self.x_compare[i]-self.x_prev[2]))


        min_value = min(tab)
        self.i = tab.index(min_value)
        print('self.i', self.i)


        # pass

    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):

        self.choose_model(x)
        # print('self.i',self.i)
        q = x[:2]
        # print('qqqq',q)
        q_dot = x[2:]
        q1, q2, q1_dot, q2_dot = x

        Kd = -23
        Kp = -48
        q1_dot_delta = q1_dot - q_r_dot[0]
        q2_dot_delta = q2_dot - q_r_dot[1]
        q1_delta = q1 - q_r[0]
        q2_delta = q2 - q_r[1]
        v = np.array([q_r_ddot[0] + Kd * q1_dot_delta + Kp * q1_delta,
                      q_r_ddot[1] + Kd * q2_dot_delta + Kp * q2_delta])

        # v = q_r_ddot

        # v = q_r_ddot # TODO: add feedback
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        self.u_prev=M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        # print('uuuuuu',u)
        self.x_prev = x
        return u
