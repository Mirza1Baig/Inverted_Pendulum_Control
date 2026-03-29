"""
Cart-Pole Project 
========================
Goal: Stabilize the pole for >60s starting from a 9-degree tilt.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation
from matplotlib.gridspec import GridSpec

# -- Physical Constants --
G_ACCEL = 9.81
M_CART = 1.0
M_POLE = 0.1
P_LEN = 1.0
TRACK_W = 4.0
H_TRACK = TRACK_W / 2

DT = 0.006
F_STEPS = 3
MAX_F = 400.0

# -- Manually Tuned constants --
# Bumped up base_f to 255 to catch the initial fall more aggressively, tune p_curve for feather balancing
base_f = 255.0
p_curve = 0.72
f_extra = 0.09
k_angle = 68.0
k_accel = 5.0
drift_b = 1.0
damp_v = 30.0
edge_m = 0.10 
START_DEG = 9.0



def get_accel(state, F):
    _, x_dot, th, th_dot = state
    s, c = np.sin(th), np.cos(th)
    m_sum = M_CART + M_POLE
    
    # Calculating angular and linear acceleration
    tmp = (F + M_POLE * P_LEN * th_dot**2 * s) / m_sum
    th_acc = (G_ACCEL * s - c * tmp) / (P_LEN * (4/3 - M_POLE * c**2 / m_sum))
    x_acc = tmp - M_POLE * P_LEN * th_acc * c / m_sum
    return np.array([x_dot, x_acc, th_dot, th_acc])


def rk4_step(state, F):
    # RK4 integration for better stability than Euler
    k1 = get_accel(state, F)
    k2 = get_accel(state + DT/2 * k1, F)
    k3 = get_accel(state + DT/2 * k2, F)
    k4 = get_accel(state + DT * k3, F)
    return state + DT/6 * (k1 + 2*k2 + 2*k3 + k4)


# -- Control Logic -----------------

class MyController:
    def __init__(self):
        self.last_th_v = 0.0

    def reset(self):
        self.last_th_v = 0.0

    def get_thrust(self, state):
        x, x_v, th, th_v = state

       
        th_a_est = (th_v - self.last_th_v) / DT
        self.last_th_v = th_v

        sin_th = abs(np.sin(th))

        # Main balancing force
        f_main = base_f * (sin_th ** p_curve)
        f_plus = f_main * f_extra
        f_ff = k_accel * th_a_est * sin_th
        mag = f_main + f_plus + abs(f_ff)

        # Centering bias
        to_mid = -np.sign(x) if abs(x) > 1e-4 else 0.0
        p_side = np.sign(th) if abs(th) > 1e-6 else 0.0
        
        ratio = min(abs(x) / H_TRACK, 1.0)
        eff_bias = 1.0 + (drift_b - 1.0) * ratio
        
        heading_in = (p_side == to_mid)
        mult = eff_bias if heading_in else (1.0 / eff_bias)

        f_out = p_side * mag * mult

        # Wall safety guard
        dist_w = H_TRACK - abs(x)
        w_side = np.sign(x)
        too_close = dist_w < edge_m
        
        f_g = 0.0
        if too_close:
            prox = (edge_m / max(dist_w, 0.005)) ** 2
            g_mag = min(base_f * 1.8 * prox, MAX_F * 0.9)
            
            if p_side == w_side:
                w = min(prox * 0.6, 1.0)
                f_out = f_out * (1 - w) + (-w_side * g_mag) * w
            else:
                f_g = -w_side * g_mag * 0.3

        # Dampening to stop oscillations
        d_th = -k_angle * th_v * sin_th
        d_x = -damp_v * x_v * 0.22

        total = f_out + f_g + d_th + d_x
        return float(np.clip(total, -MAX_F, MAX_F))


# -- Simulation State --

class MySim:
    def __init__(self):
        self.brain = MyController()
        self.reset()

    def reset(self):
        rad0 = np.radians(START_DEG)
        self.state = np.array([0.0, 0.0, rad0, 0.0])
        self.time = 0.0
        self.active = True
        self.brain.reset()
        self.h_t = []
        self.h_th = []
        self.h_x = []

    def step(self):
        if not self.active:
            return
        F = self.brain.get_thrust(self.state)
        self.state = rk4_step(self.state, F)
        
        # Boundary bounce
        if abs(self.state[0]) >= H_TRACK - 0.005:
            self.state[0] = np.sign(self.state[0]) * (H_TRACK - 0.005)
            self.state[1] *= -0.1
        
        if abs(self.state[2]) > np.pi / 2:
            self.active = False
            
        self.time += DT
        self.h_t.append(self.time)
        self.h_th.append(np.degrees(self.state[2]))
        self.h_x.append(self.state[0])


# -- Visualization -------------------------------------------------------

def main():
    sim = MySim()

    COL_BG = "white"
    COL_AX = "#f0f0f0"
    COL_P = "blue"
    COL_C = "green"

    fig = plt.figure(figsize=(12, 7), facecolor="silver")
    fig.canvas.manager.set_window_title("Cart-pole stability(PID controls)")

    gs = GridSpec(
        2, 2,
        figure=fig,
        left=0.06, right=0.94,
        top=0.90, bottom=0.10,
        hspace=0.4, wspace=0.3,
        height_ratios=[1.5, 1.0],
    )

    ax1 = fig.add_subplot(gs[0, :], facecolor=COL_BG)
    ax2 = fig.add_subplot(gs[1, 0], facecolor=COL_BG)
    ax3 = fig.add_subplot(gs[1, 1], facecolor=COL_BG)

    fig.suptitle("Cart-Pole Stability(Feather balancing)", color="black", fontsize=14)

    for a in [ax1, ax2, ax3]:
        a.tick_params(labelsize=8)
        a.grid(True, color="#ddd", linestyle="--")

    # Animation Frame
    ax1.set_xlim(-2.4, 2.4)
    ax1.set_ylim(-0.5, 2.0)
    ax1.set_aspect("equal")
    ax1.set_xticks([])
    ax1.set_yticks([])

    # Track components
    ax1.plot([-H_TRACK, H_TRACK], [0, 0], color="black", lw=3)
    for s in [-1, 1]:
        ax1.plot([s * H_TRACK]*2, [-0.15, 0.15], color="red", lw=3)
    ax1.plot([0, 0], [-0.15, 0.15], color="gray", lw=1, ls="--")

    # Physical objects
    cart = mpatches.FancyBboxPatch(
        (-0.28, -0.11), 0.56, 0.22,
        boxstyle="round,pad=0.02",
        facecolor="darkgray", edgecolor="black", lw=1)
    ax1.add_patch(cart)

    w1 = plt.Circle((-0.15, -0.11), 0.07, facecolor="black")
    w2 = plt.Circle(( 0.15, -0.11), 0.07, facecolor="black")
    ax1.add_patch(w1)
    ax1.add_patch(w2)

    p_line, = ax1.plot([], [], lw=6, color=COL_P, solid_capstyle="round")
    p_bob,  = ax1.plot([], [], "o", ms=12, color=COL_P)

    # Status labels
    txt_th = ax1.text(-2.3, 1.8, "", color=COL_P, fontweight="bold")
    txt_x = ax1.text(-2.3, 1.6, "", color=COL_C, fontweight="bold")
    txt_msg = ax1.text(0, 1.9, "", color="black", ha="center")

    # Plot lines
    ax2.set_ylabel("Theta (deg)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylim(-35, 35)
    line_th, = ax2.plot([], [], color=COL_P)

    ax3.set_ylabel("X-Pos (m)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylim(-2.2, 2.2)
    line_x, = ax3.plot([], [], color=COL_C)

    f_handle = [None]

    def update(f):
        for _ in range(F_STEPS):
            if sim.active:
                sim.step()

        x_cur, _, th_cur, _ = sim.state
        deg_cur = np.degrees(th_cur)

        # Coordinates
        tip_x, tip_y = x_cur + np.sin(th_cur)*0.88, 0.11 + np.cos(th_cur)*0.88
        
        cart.set_x(x_cur - 0.28)
        cart.set_y(-0.11)
        w1.center, w2.center = (x_cur - 0.15, -0.11), (x_cur + 0.15, -0.11)

        p_line.set_data([x_cur, tip_x], [0.11, tip_y])
        p_bob.set_data([tip_x], [tip_y])

        txt_th.set_text(f"Angle: {deg_cur:+.1f}")
        txt_x.set_text(f"Pos: {x_cur:+.2f}")

        if not sim.active:
            txt_msg.set_text("SIMULATION HALTED")
            txt_msg.set_color("red")
        else:
            txt_msg.set_text("ACTIVE")
        window_sec = 10.0  # Shows 10 seconds of real-world time
        
        if len(sim.h_t) > 2:
            t_calibrated = np.array(sim.h_t) * 3.0
            current_time = t_calibrated[-1]
            
            mask = t_calibrated > (current_time - window_sec)
            
            line_th.set_data(t_calibrated[mask], np.array(sim.h_th)[mask])
            line_x.set_data(t_calibrated[mask], np.array(sim.h_x)[mask])
            
            ax2.set_xlim(max(0, current_time - window_sec), max(window_sec, current_time))
            ax3.set_xlim(max(0, current_time - window_sec), max(window_sec, current_time))
            
        return cart, w1, w2, p_line, p_bob, txt_th, txt_x

    ani = animation.FuncAnimation(fig, update, interval=18, blit=False)
    plt.show()

if __name__ == "__main__":
    main()
