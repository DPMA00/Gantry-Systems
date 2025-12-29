import matplotlib.pyplot as plt
import numpy as np

def plot2D(xc,xl,zl,Ts):
    plt.ion()
    fig, ax = plt.subplots()

 
    ax.set_xlim(-5, 5)
    ax.set_ylim(-1, 0.1)
    ax.grid(True)

    cart_point, = ax.plot([], [], 'ks', markersize=20) 
    cable_line, = ax.plot([], [], '-', linewidth=2) 
    load_point, = ax.plot([], [], 'ro', markersize=15) 


    ax.plot([-5, 5], [0, 0], 'k--', linewidth=1)

    step = max(1, len(xc)//500) 

    for i in range(0, len(xc), step):
        # cart at (x, 0)
        cart_point.set_data([xc[i]], [0.0])

        # cable from cart to payload
        cable_line.set_data([xc[i], xl[i]], [0.0, zl[i]])

        # payload
        load_point.set_data([xl[i]], [zl[i]])

        fig.canvas.draw_idle()
        plt.pause(Ts)

    plt.ioff()
    plt.show()