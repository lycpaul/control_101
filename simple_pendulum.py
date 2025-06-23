import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Pendulum parameters
L = 1.0  # length of the arm (m)
m = 1.0  # mass of the arm (kg)
g = 9.81  # gravity (m/s^2)

# Calculate moment of inertia for a rod with mass at center
# For a uniform rod of length L with mass m, I = mL^2/12
I = m * L**2 / 12

# Time parameters
dt = 0.1
t_max = 2 * np.pi  # one full cycle
time = np.linspace(0, t_max, int(t_max / dt))

# Define the desired motion (simple sinusoidal swing)
theta_max = np.pi  # maximum swing angle (180 degrees)
omega = 1.0  # angular frequency (rad/s)

# Calculate theta, omega, and alpha as functions of time
theta = theta_max * np.sin(omega * time)
theta_dot = theta_max * omega * np.cos(omega * time)  # angular velocity
theta_ddot = -theta_max * omega**2 * np.sin(omega * time)  # angular acceleration

# Calculate required torque using equation of motion
# τ = I*α + mg*(L/2)*sin(θ)
# where I*α is the inertial torque and mg*(L/2)*sin(θ) is the gravitational torque
torque_inertial = I * theta_ddot
torque_gravity = m * g * (L / 2) * np.sin(theta)
torque_total = torque_inertial + torque_gravity

# Create figure with subplots
fig = plt.figure(figsize=(12, 5))
ax1 = plt.subplot(1, 2, 1)  # Pendulum animation
ax2 = plt.subplot(1, 2, 2)  # Torque plot

# Setup pendulum plot
ax1.set_xlim(-1.2, 1.2)
ax1.set_ylim(-1.2, 1.2)
ax1.set_aspect("equal")
ax1.grid(True)
ax1.set_title("Pendulum Motion")
ax1.set_xlabel("X (m)")
ax1.set_ylabel("Y (m)")

# Pendulum line and mass
(line,) = ax1.plot([], [], "k-", lw=3, label="Rod")
(mass_point,) = ax1.plot([], [], "ro", markersize=10, label="Mass (CG)")
(pivot_point,) = ax1.plot(0, 0, "ko", markersize=8, label="Pivot")
ax1.legend()

# Setup torque plot
ax2.plot(time, torque_total, "b-", linewidth=2, label="Total Torque")
ax2.plot(time, torque_inertial, "r--", linewidth=1, label="Inertial Torque")
ax2.plot(time, torque_gravity, "g--", linewidth=1, label="Gravity Torque")
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("Torque (N⋅m)")
ax2.set_title("Required Torque vs Time")
ax2.grid(True)
ax2.legend()

# Add vertical line to show current time
time_line = ax2.axvline(x=0, color="k", linestyle=":", alpha=0.7)


def init():
    line.set_data([], [])
    mass_point.set_data([], [])
    return line, mass_point


def animate(frame):
    # Get current time index
    t_idx = frame % len(time)
    current_time = time[t_idx]
    current_theta = theta[t_idx]

    # Calculate pendulum position
    x_end = L * np.sin(current_theta)
    y_end = -L * np.cos(current_theta)

    # Mass position (at center of rod)
    x_mass = (L / 2) * np.sin(current_theta)
    y_mass = -(L / 2) * np.cos(current_theta)

    # Update pendulum
    line.set_data([0, x_end], [0, y_end])
    mass_point.set_data([x_mass], [y_mass])

    # Update time line on torque plot
    time_line.set_xdata([current_time, current_time])

    return line, mass_point, time_line


# Create animation
ani = animation.FuncAnimation(
    fig, animate, frames=len(time), init_func=init, blit=True, interval=50, repeat=True
)

plt.tight_layout()

# Save animation as GIF
print("Saving animation as GIF...")
ani.save('pendulum_torque_animation.gif', writer='pillow', fps=20, dpi=100)
print("Animation saved as 'pendulum_torque_animation.gif'")

plt.show()

# Print some information about the system
print(f"Pendulum Parameters:")
print(f"Length: {L} m")
print(f"Mass: {m} kg")
print(f"Moment of Inertia: {I:.4f} kg⋅m²")
print(f"Maximum swing angle: {theta_max*180/np.pi:.1f}°")
print(f"Maximum torque: {np.max(np.abs(torque_total)):.3f} N⋅m")
