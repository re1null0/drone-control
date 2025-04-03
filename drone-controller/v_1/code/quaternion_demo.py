import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

# **Original Point**
point = np.array([0, 0, 1])

# **Define Quaternion Rotations**
# We define four rotations:
# 1. 90° about the x-axis
# 2. 90° about the y-axis
# 3. 90° about the z-axis
# 4. 180° about the x-axis
rotations = {
    "Rotation 1: 90° about x-axis": R.from_euler('x', 90, degrees=True),
    "Rotation 2: 90° about y-axis": R.from_euler('y', 90, degrees=True),
    "Rotation 3: 90° about z-axis": R.from_euler('z', 90, degrees=True),
    "Rotation 4: 180° about x-axis": R.from_euler('x', 180, degrees=True)
}

# **Apply Rotations and Print Results**
print("Quaternion Rotation Examples:")
print("--------------------------------")
for desc, rot in rotations.items():
    # SciPy returns quaternions in the order [x, y, z, w]
    q_xyzw = rot.as_quat()
    # Reorder to [w, x, y, z]
    q_wxyz = np.roll(q_xyzw, 1)
    
    # Apply the quaternion rotation to the point
    rotated_point = rot.apply(point)
    
    print(desc)
    print("  Quaternion [w, x, y, z]:", np.round(q_wxyz, 3))
    print("  Rotated Point:         ", np.round(rotated_point, 3))
    print("--------------------------------")

# **Optional: 3D Visualization**
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Plot the original point
ax.scatter(point[0], point[1], point[2], color='black', s=100, label='Original (0,0,1)')

# Colors for the rotated points
colors = ['red', 'green', 'blue', 'purple']
for (desc, rot), color in zip(rotations.items(), colors):
    rotated_point = rot.apply(point)
    ax.scatter(rotated_point[0], rotated_point[1], rotated_point[2],
               color=color, s=100, label=desc)

# Configure the axes
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Quaternion Rotations of Point (0,0,1)')
ax.legend()
plt.show()