
"""quaternion_demo_slider.py

Interactive quadrotor orientation visualiser (compatible with matplotlib 3.2).

* Body‑frame: x front, y left, z up.
* Arms: front red, rear white.
* Props: grey.
* Legs: black.

Controls
--------
Move the *i, j, k, w* sliders (range [‑1, 1]).
The quaternion is re‑normalised automatically.

Usage
-----
$ python quaternion_demo_slider.py
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from scipy.spatial.transform import Rotation as R

# -----------------------------------------------------------------------------#
# Geometry
# -----------------------------------------------------------------------------#
ARM_RADIUS = 0.225        # Diagonal arm length (m)
LEG_LENGTH = 0.050        # Simple vertical legs
PROP_MARKER_SIZE = 250

# -----------------------------------------------------------------------------#
# Helper functions
# -----------------------------------------------------------------------------#
def body_motor_positions(radius: float) -> np.ndarray:
    """Return 4×3 array of motor positions in body frame."""
    dirs = np.array([[1,  1, 0],
                     [1, -1, 0],
                     [-1,  1, 0],
                     [-1, -1, 0]], dtype=float) / np.sqrt(2)
    return dirs * radius


def set_equal_aspect(ax):
    """Approximate equal aspect for older matplotlib (<3.4)."""
    xlim = ax.get_xlim3d()
    ylim = ax.get_ylim3d()
    zlim = ax.get_zlim3d()
    ranges = [abs(lim[1] - lim[0]) for lim in (xlim, ylim, zlim)]
    max_range = max(ranges)
    mids = [np.mean(lim) for lim in (xlim, ylim, zlim)]
    for mid, setlim in zip(mids, (ax.set_xlim3d, ax.set_ylim3d, ax.set_zlim3d)):
        setlim(mid - max_range / 2, mid + max_range / 2)


def plot_drone(ax, quat):
    """Draw the quadrotor at the given quaternion (i, j, k, w)."""
    ax.cla()  # clear

    # Geometry in body frame
    centre = np.zeros((1, 3))
    motors_b = body_motor_positions(ARM_RADIUS)

    # Rotate to world
    rot = R.from_quat(quat)
    centre_w = rot.apply(centre)[0]
    motors_w = rot.apply(motors_b)

    # Arms + rotors
    for m in motors_w:
        arm_col = 'red' if m[0] > 0 else 'white'
        ax.plot([centre_w[0], m[0]], [centre_w[1], m[1]],
                [centre_w[2], m[2]], color=arm_col, linewidth=3)
        ax.scatter(m[0], m[1], m[2], s=PROP_MARKER_SIZE,
                   c='gray', edgecolors='black', depthshade=True)
        # Leg
        leg_end = m - np.array([0, 0, LEG_LENGTH])
        ax.plot([m[0], leg_end[0]], [m[1], leg_end[1]],
                [m[2], leg_end[2]], color='black', linewidth=2)

    # Body axes
    axis_len = ARM_RADIUS * 1.2
    axes_b = np.eye(3) * axis_len
    cols = ['red', 'green', 'blue']
    labels = ['x', 'y', 'z']
    for vec_b, c, lab in zip(axes_b, cols, labels):
        vec_w = rot.apply(vec_b)
        ax.quiver(centre_w[0], centre_w[1], centre_w[2],
                  vec_w[0], vec_w[1], vec_w[2],
                  color=c, linewidth=2)
        ax.text(centre_w[0] + vec_w[0],
                centre_w[1] + vec_w[1],
                centre_w[2] + vec_w[2],
                lab, color=c)

    # Aesthetics
    lim = ARM_RADIUS * 1.5
    ax.set_xlim([-lim, lim])
    ax.set_ylim([-lim, lim])
    ax.set_zlim([-0.2 * lim, 1.2 * lim])
    ax.set_xlabel('X (front)')
    ax.set_ylabel('Y (left)')
    ax.set_zlabel('Z (up)')
    set_equal_aspect(ax)
    ax.view_init(elev=25, azim=135)
    ax.set_title(f'Quaternion (i j k w) = '
                 f'{quat[0]:.2f}, {quat[1]:.2f}, {quat[2]:.2f}, {quat[3]:.2f}')


# -----------------------------------------------------------------------------#
# Interactive figure
# -----------------------------------------------------------------------------#
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Slider axes (left, bottom, width, height)
slider_axes = []
labels = ['i', 'j', 'k', 'w']
for idx in range(4):
    rect = [0.15, 0.02 + idx * 0.03, 0.70, 0.02]
    slider_axes.append(plt.axes(rect))

sliders = [Slider(axs, lab, -1.0, 1.0, valinit=0.0) for axs, lab in zip(slider_axes, labels)]
sliders[-1].set_val(1.0)   # initialise w = 1

def on_slider_change(val):
    raw = np.array([s.val for s in sliders], dtype=float)
    if np.linalg.norm(raw) == 0:
        raw[3] = 1.0  # fall back to identity
    quat = raw / np.linalg.norm(raw)
    plot_drone(ax, quat)
    fig.canvas.draw_idle()

for s in sliders:
    s.on_changed(on_slider_change)

# Initial draw
on_slider_change(None)

plt.show()
