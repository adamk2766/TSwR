from copy import copy
import numpy as np


class ESO:
    def __init__(self, A, B, W, L, state, Tp):
        self.A = A
        self.B = B
        self.W = W
        self.L = L
        self.state = np.pad(np.array(state), (0, A.shape[0] - len(state)))
        self.Tp = Tp
        self.states = []

        if len(state) == 2:
            self.len_state = 1
        else:
            self.len_state = 2

    def set_B(self, B):
        self.B = B

    def update(self, q, u):
        self.states.append(copy(self.state))
        # print('self.len_state',self.len_state)
        # print('self.A @ self.state ',self.A @ self.state )
        # print('self.state ', self.state)
        # print('self.A ', self.A)
        # print('np.array( (self.B @ [u])[0]).T',np.array( (self.B @ [u])[0]).T)
        # print('self.B ',self.B )
        # print('u ', u)
        # print('self.L @  np.array( q - self.state[0:self.len_state]).T', self.L @  np.array( q - self.state[0:self.len_state]).T)

        # print('self.A @ self.state + self.B @ [u]', self.A @ self.state + np.array( (self.B @ [u])[0]).T)

        # print(' q - self.state ',  [ np.array( q - self.state[0:self.len_state]).T])
        # print(' self.L ', self.L)
        # print(' q ', q)
        # print('self.W', self.W)
        # print('self.state ',self.state[0:self.len_state])
        # print(self.B @ u )
        # z_d_dot=self.A @ self.state + self.B @ [u] + self.L @ [( q - self.W @ self.state)]

        # print('z====>>>>>>>>>>>>>>>>>>>>>>>>>>>>>1',  z_d_dot)
        z_d_dot=self.A @ self.state + np.array( (self.B @ [u])[0]).T + self.L @  np.array( q - self.state[0:self.len_state]).T
        # print('z====>>>>>>>>>>>>>>>>>>>>>>>>>>>>>2', z_d_dot)
        # print('z_dot',z_d_dot)
        z_d = self.state + (z_d_dot * self.Tp)
        self.state= np.array(z_d).flatten()
        # print('z====>>>>>>>>>>>>>>>>>>>>>>>>>>>>>',np.array(self.state).T)


        ### TODO implement ESO update

    def get_state(self):
        return self.state
