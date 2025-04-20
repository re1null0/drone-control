import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

from flightsim.simulate import Quadrotor, simulate
from flightsim.drone_params import quad_params
import se3_control
import waypoint_traj

# ---------- Utility Functions ----------

def compute_roll_from_quaternion(q):
    # Assuming Z-X-Y Euler angles with quaternion [x, y, z, w],
    # we extract roll (phi). This formula may vary with conventions.
    r = Rotation.from_quat(q)
    # Using intrinsic ZXY, extract Euler angles
    phi, theta, psi = r.as_euler('ZXY', degrees=False)
    return phi

def estimate_settling_time(time, response, final_value, tol=0.02):
    # Settling time: time after which response remains within tol*final_value
    err = np.abs(response - final_value)
    idx = np.where(err > tol * np.abs(final_value))[0]
    if len(idx) == 0:
        return 0
    return time[idx[-1]]  # last time outside tolerance

# ---------- Simulation Settings ----------

# Use a hover trajectory
points = np.array([
    [0, 0, 0],
    [0, 0, 0],  # constant hover for testing
])
traj = waypoint_traj.WaypointTraj(points)

T_final = 10.0
time_array = np.linspace(0, T_final, int(T_final*50))  # 50 Hz sample

# ---------- Scenario 1: Lateral (x) Perturbation ----------

# Create a lateral step: initial x is offset from desired (0)
init_state_pos = {
    'x': np.array([-1.0, 0, 0.25]),   # 1 m step in x
    'v': np.zeros(3),
    'q': np.array([0, 0, 0, 1]),
    'w': np.zeros(3)
}

quadrotor = Quadrotor(quad_params)
controller_pos = se3_control.SE3Control(quad_params)
# (Assume controller gains are already tuned.)
(time_pos, state_pos, control_pos, flat_pos, exit_pos) = simulate(init_state_pos, quadrotor, controller_pos, traj, T_final)

# Plot x position vs. time
fig1, ax1 = plt.subplots(figsize=(8,4))
ax1.plot(time_pos, flat_pos['x'][:,0], 'k-', label='Desired x')
ax1.plot(time_pos, state_pos['x'][:,0], 'r.', label='Actual x')
ax1.set_title('Lateral (x) Step Response')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('x Position (m)')
ax1.legend()
ax1.grid(True)

# Estimate metrics (example estimation: final value should be near 0, initial error is 1 m)
final_x = flat_pos['x'][-1,0]
settling_time_x = estimate_settling_time(time_pos, state_pos['x'][:,0], final_x)
print(f"Approximate lateral settling time: {settling_time_x:.2f} s")

# ---------- Scenario 2: Roll Angle Perturbation ----------

# Create a roll perturbation by initializing with a small roll offset.
# For a small roll, approximate quaternion: [sin(phi/2), 0, 0, cos(phi/2)].
phi0 = np.deg2rad(10)  # 10 degree roll perturbation
q_roll = np.array([0, 0.5, 0, 1])

init_state_roll = {
    'x': np.array([0, 0, 0]),   # desired hover position
    'v': np.zeros(3),
    'q': q_roll,
    'w': np.zeros(3)
}

quadrotor2 = Quadrotor(quad_params)
controller_roll = se3_control.SE3Control(quad_params)
(time_roll, state_roll, control_roll, flat_roll, exit_roll) = simulate(init_state_roll, quadrotor2, controller_roll, traj, T_final)
# Extract roll angle over time from the quaternion output (actual attitude)
roll_angles = np.array([compute_roll_from_quaternion(q) for q in state_roll['q']])
# For desired, assume zero roll
desired_roll = np.zeros_like(roll_angles)

fig2, ax2 = plt.subplots(figsize=(8,4))
ax2.plot(time_roll, np.rad2deg(desired_roll), 'k-', label='Desired Roll (deg)')
ax2.plot(time_roll, np.rad2deg(roll_angles), 'b.', label='Actual Roll (deg)')
ax2.set_title('Roll Angle Step Response')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Roll Angle (deg)')
ax2.legend()
ax2.grid(True)

final_roll = desired_roll[-1]
settling_time_roll = estimate_settling_time(time_roll, roll_angles, final_roll)
print(f"Approximate roll settling time: {settling_time_roll:.2f} s")
    
plt.show()