import numpy as np
import casadi as ca
from time import time


class NMPC:
    def __init__(self, n, m, N, Ts, Q, R):
        self.n = n
        self.m = m
        self.N = N
        self.Ts =  Ts
        self.Q = Q
        self.R = R
        
        self.X = ca.SX.sym('X',n, N+1) # N+1 State predictions
        self.U = ca.SX.sym('U',m, N) # N control predictions
        
        self.nrDynamicObstacles = 2
    
    def Constraints(self, upperXs,lowerXs,upperUs,lowerUs):
        self.upperXs = upperXs
        self.lowerXs = lowerXs
        self.upperUs = upperUs
        self.lowerUs = lowerUs
        
    

        
    def createModel(self, states, controls, RHS):
        self.f = ca.Function('f', [states,controls], [RHS])
        
    def sim_step(self, x0, t, Ts, u, f):
        #Simulates the MPC one time step

        fval = self.f(x0, u[:,0]) # first control is applied to RHS
        st = ca.DM.full(x0 + (Ts*fval)) #explicit shift to next state
        
        t = t + Ts
       
        return t, st
    
        
        
    def PointCtrl(self):
        # Iinitialize a symbolic parameter vector
        self.P = ca.SX.sym('P', self.n + self.n) 
        

        
        ### Multiple Shooting ###
        self.g = self.X[:, 0] - self.P[:self.n]  # Initial state constraint
        self.obj = 0  # Initialize objective function
        
        for k in range(self.N):
            st = self.X[:, k]
            con = self.U[:, k]
            self.obj += (st - self.P[self.n:]).T @ self.Q @ (st - self.P[self.n:]) #target tracking penalty
            self.obj += con.T @ self.R @ con
            st_next = self.X[:, k+1]
            k1 = self.f(st, con)
            k2 = self.f(st + self.Ts/2*k1, con)
            k3 = self.f(st + self.Ts/2*k2, con)
            k4 = self.f(st + self.Ts * k3, con)
            RK4 = st + (self.Ts / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
            self.g = ca.vertcat(self.g, st_next - RK4)
        

        
        ### NLP Parameters ###
        self.opt_var = ca.vertcat(self.X.reshape((-1, 1)), self.U.reshape((-1, 1))) 
        
        self.nlp_prob = {
            'f': self.obj,
            'x': self.opt_var,
            'g': self.g,
            'p': self.P
        }
        
        # Solver options
        self.opts = {
            'ipopt': {
                'max_iter': 2000,
                'print_level': 0,
                'acceptable_tol': 1e-8,
                'acceptable_obj_change_tol': 1e-6
            },
            'print_time': 0
        }
        
        # Set up the NLP solver
        self.solver = ca.nlpsol('solver', 'ipopt', self.nlp_prob, self.opts)
        
        ### Constraint Bounds ###
        total_constraints = self.n * (self.N + 1)
        self.lbg = ca.DM.zeros((total_constraints, 1))
        self.ubg = ca.DM.zeros((total_constraints, 1))
        
        # Dynamics constraints
        self.lbg[:self.n * (self.N + 1)] = 0
        self.ubg[:self.n * (self.N + 1)] = 0
        
        # Collision constraints
        self.lbg[self.n * (self.N + 1):] = 4  # Minimum distance squared (r+r_obstacle)**2
        self.ubg[self.n * (self.N + 1):] = ca.inf
        
        # State and control bounds
        self.lb_state_const = ca.DM.zeros((self.n * (self.N + 1), 1))
        self.ub_state_const = ca.DM.zeros((self.n * (self.N + 1), 1))
        self.lb_cont_const = ca.DM.zeros((self.m * self.N, 1))
        self.ub_cont_const = ca.DM.zeros((self.m * self.N, 1))
        
        # States
        for i in range(self.n):
            self.lb_state_const[i: self.n * (self.N + 1): self.n] = self.lowerXs[i]    
            self.ub_state_const[i: self.n * (self.N + 1): self.n] = self.upperXs[i] 
        
        # Controls
        for i in range(self.m):
            self.lb_cont_const[i: self.m * self.N: self.m] = self.lowerUs[i]  
            self.ub_cont_const[i: self.m * self.N: self.m] = self.upperUs[i]  
    
        self.lbx = ca.vertcat(self.lb_state_const, self.lb_cont_const)
        self.ubx = ca.vertcat(self.ub_state_const, self.ub_cont_const)
    
        self.args = {
            'lbg': self.lbg,  # constraints lower bound
            'ubg': self.ubg,  # constraints upper bound
            'lbx': self.lbx,
            'ubx': self.ubx
        }
        
        
    def stZeroRef(self, state_init, x_ref=None):
        self.state_init = state_init
        self.x_ref = x_ref
        
        # Runtime constants and other information
        self.t0 = 0
        self.mpc_iter = 0
        self.u_init = ca.DM.zeros((self.m,self.N))
        
        self.t = ca.DM(self.t0)  # time history
        self.X0 = ca.repmat(self.state_init, 1, self.N+1)
        self.times = np.array([[0]])
        
    
    
    def solveProblem(self):
        # Update the parameter vector `p` with current state, reference
        self.args['p'] = ca.vertcat(
            self.state_init,
            self.x_ref,
        )
        
        # Warm-start the solver with the previous solution
        self.args['x0'] = ca.vertcat(
            ca.reshape(self.X0, self.n*(self.N+1), 1),  # Warm-started states
            ca.reshape(self.u_init, self.m*self.N, 1)   # Warm-started controls
        )
        
        # Solve the NLP
        sol = self.solver(
            x0=self.args['x0'],  # Initial guess (warm-started)
            lbx=self.args['lbx'],
            ubx=self.args['ubx'],
            lbg=self.args['lbg'],
            ubg=self.args['ubg'],
            p=self.args['p']
        )
        
        # Extract the optimal solution
        u_opt = ca.reshape(sol['x'][self.n * (self.N + 1):], self.m, self.N)
        X0_opt = ca.reshape(sol['x'][: self.n * (self.N+1)], self.n, self.N+1)  
        
        self.horizon = X0_opt 
                

        self.X0 = ca.horzcat(
            X0_opt[:, 1:],
            ca.reshape(X0_opt[:, -1], -1, 1) 
        )
        self.u_init = ca.horzcat(
            u_opt[:, 1:],  
            ca.reshape(u_opt[:, -1], -1, 1)  
        )
        
        self.t0, self.state_init = self.sim_step(self.state_init, self.t0, self.Ts, u_opt, self.f)
        
        self.mpc_iter += 1
        print(self.mpc_iter)
        
        return self.state_init, u_opt[:, 0], self.t0
    
    