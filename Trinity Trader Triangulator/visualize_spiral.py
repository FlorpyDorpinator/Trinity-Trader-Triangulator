import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Create figure
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# Function to convert H,V to sphere coordinates
def sphere_coords(h, v):
    h_rad = np.radians(h)
    v_rad = np.radians(v)
    x = np.cos(v_rad) * np.cos(h_rad)
    y = np.cos(v_rad) * np.sin(h_rad)
    z = np.sin(v_rad)
    return x, y, z

# Spiral 1: V starts at 0°
t1 = np.arange(0, 1178, 1)
h1 = t1 % 360
v1 = t1 * 0.0764
x1, y1, z1 = sphere_coords(h1, v1)
ax.plot(x1, y1, z1, 'b-', linewidth=2, label='Spiral 1 (V=0°)', alpha=0.7)

# Spiral 2: V starts at 27.5°
t2 = np.arange(0, 818, 1)
h2 = t2 % 360
v2 = 27.5 + t2 * 0.0764
x2, y2, z2 = sphere_coords(h2, v2)
ax.plot(x2, y2, z2, 'g-', linewidth=2, label='Spiral 2 (V=27.5°)', alpha=0.7)

# Spiral 3: V starts at 55°
t3 = np.arange(0, 458, 1)
h3 = t3 % 360
v3 = 55 + t3 * 0.0764
x3, y3, z3 = sphere_coords(h3, v3)
ax.plot(x3, y3, z3, 'r-', linewidth=2, label='Spiral 3 (V=55°)', alpha=0.7)

# Spiral 4: V starts at 82.5°, goes down
t4 = np.arange(0, 1080, 1)
h4 = t4 % 360
v4 = 82.5 - t4 * 0.0764
x4, y4, z4 = sphere_coords(h4, v4)
ax.plot(x4, y4, z4, 'orange', linewidth=2, label='Spiral 4 (V=82.5° down)', alpha=0.7)

# Target point: H=282°, V=66°
xt, yt, zt = sphere_coords(282, 66)
ax.scatter([xt], [yt], [zt], color='red', s=200, marker='*', 
           label='Target (282°, 66°)', edgecolors='black', linewidths=2)

# Draw sphere wireframe
u = np.linspace(0, 2 * np.pi, 30)
v = np.linspace(0, np.pi, 20)
x_sphere = np.outer(np.cos(u), np.sin(v))
y_sphere = np.outer(np.sin(u), np.sin(v))
z_sphere = np.outer(np.ones(np.size(u)), np.cos(v))
ax.plot_wireframe(x_sphere, y_sphere, z_sphere, color='gray', alpha=0.1, linewidth=0.5)

# Labels and settings
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Satellite Dish Spiral Coverage Pattern\n(27.5° beam width beam not shown)', fontsize=14)
ax.legend(loc='upper right')
ax.set_box_aspect([1,1,1])

plt.tight_layout()
plt.show()
