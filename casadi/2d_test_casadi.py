from NMPC import NMPC
import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from plotter import *

def get_p(q):
    q = np.array(q).squeeze()
    x,l,theta,_,_,_ = q
    return np.array([x+l*np.sin(theta), -l*np.cos(theta)])
    
def main():
    nx = 6
    nu = 2
    N = 30
    Ts = 0.1
    
    Q = ca.diagcat(200,400,200,200,200,200)
    R = ca.diagcat(1,1)
    
    MPC = NMPC(nx,nu,N,Ts,Q,R)
    
    x = ca.SX.sym('x')
    l = ca.SX.sym('l')
    theta = ca.SX.sym('theta')
    x_dot = ca.SX.sym('x_dot')
    l_dot = ca.SX.sym('l_dot')
    th_dot=ca.SX.sym('theta_dot')
    
    F = ca.SX.sym('F')
    ul = ca.SX.sym('ul')
    
    states = ca.vertcat(x,l,theta,x_dot,l_dot,th_dot)
    qdot = ca.vertcat(x_dot,l_dot,th_dot)
    
    Q_ = ca.vertcat(F,ul, 0)
    controls = ca.vertcat(F,ul)
    
    g = 9.81
    m1 = 2
    m2 = 2.5
    c = 0.1
    
    
    J = ca.vertcat(ca.horzcat(1, ca.sin(theta),  l*ca.cos(theta)),
                ca.horzcat(0, -ca.cos(theta), l*ca.sin(theta)))
    
    J_dot = ca.vertcat(ca.horzcat(0, th_dot*ca.cos(theta),  l_dot*ca.cos(theta) - th_dot*l*ca.sin(theta)),
                    ca.horzcat(0, th_dot*ca.sin(theta),  l_dot*ca.sin(theta) + th_dot*l*ca.cos(theta)))
    
    D = ca.diag(ca.vertcat(0,0,c))
    
    M_1 = ca.diag(ca.vertcat(m1,0,0))

    A_1 = ca.SX.zeros(2,3)
    
    A_2 = ca.vertcat(ca.horzcat(0, 0, ca.cos(theta)),
                  ca.horzcat(0, 0, ca.sin(theta)))
    
    A_3 = ca.vertcat(ca.horzcat(0, ca.cos(theta), -l*ca.sin(theta)),
                  ca.horzcat(0, ca.sin(theta),  l*ca.cos(theta)))
    
    gamma_1 = qdot.T @ A_1.T @ J @ qdot
    gamma_2 = qdot.T @ A_2.T @ J @ qdot
    gamma_3 = qdot.T @ A_3.T @ J @ qdot
    Gamma = m2 * ca.vertcat(gamma_1,gamma_2,gamma_3)
    
    G = ca.vertcat(0,-m2*g*ca.cos(theta), m2*g*l*ca.sin(theta))
    
    M = M_1 + m2*(J.T @ J)
    
    Cqdot = m2 * (J_dot.T @ J + J.T @ J_dot) @ qdot - Gamma
    
    
    eps = 1e-6
    qddot = ca.solve(M+eps*ca.SX.eye(3), (Q_ - D@qdot - Cqdot - G))
    
    RHS = ca.vertcat(qdot,qddot)
    
    
    state0 = np.array([0,0.5,0,0,0,0])
    stateRef = np.array([2,0.5,0,0,0,0])
    
    MPC.stZeroRef(state0,stateRef)
    MPC.createModel(states, controls, RHS)
    
    lbx = [-5, 0.01, -np.pi/2, -2, -2, -np.pi/3]
    ubx = [5, 2, np.pi/2, 2, 2, np.pi/3]
    lbu = [-70, -200]
    ubu = [70, 0]
    
    MPC.Constraints(ubx,lbx,ubu,lbu)
    
    MPC.PointCtrl()
    
    
    simsteps = 100
    

    
    p_history = [get_p(state0)]
    state_history = [state0]
    control_history = []
    t_history = []
    
    for step in range(simsteps):
        states_sol,controls_sol,t = MPC.solveProblem()
        p_history.append(get_p(states_sol))
        control_history.append(np.array(controls_sol).squeeze())
        state_history.append(np.array(states_sol).squeeze())
        t_history.append(np.array(t).squeeze())
    
    P= np.vstack(p_history)
    X = np.vstack(state_history)
    U = np.vstack(control_history)
    xl, zl = P[:, 0], P[:, 1]
    F,ul = U[:,0], U[:,1]
    xc = X[:,0]
    #plot2D(xc,xl,zl,Ts)
    plot2D_to_video(xc,xl,zl,Ts,"controlled_dampening.mp4")
    """
    fig, ax = plt.subplots()
    ax.plot(x,y, '-o',color='r', label='closed-loop path')
    ax.axis('equal'); ax.grid(True)
    ax.legend()
    ax.set_title("MPC Closed‑Loop Simulation (100 steps)")
    ax.set_xlabel("x [m]"); plt.ylabel("y [m]")
    plt.show()
    """
    fig, ax = plt.subplots(2, 1, sharex=True)

    ax[0].step(t_history, F, where="post")
    ax[0].grid(True)
    ax[0].set_ylabel("F [N]")
    ax[0].set_title("Control inputs (stairs)")

    ax[1].step(t_history, ul, where="post")
    ax[1].grid(True)
    ax[1].set_ylabel("ul")
    ax[1].set_xlabel("time [s]")

    plt.show()


if __name__ == "__main__":
    main()