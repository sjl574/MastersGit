import numpy as np
import matplotlib.pyplot as plt

# Constants
g = 9.81  # gravity
v0 = 20.0  # initial speed in m/s
target_distance = 10.0  # meters
target_angle_deg = 30.0  # angle from forward line
target_radius = 0.20  # for hit detection

# Convert to Cartesian
angle_rad = np.radians(target_angle_deg)
target_x = target_distance * np.cos(angle_rad)
target_y = target_distance * np.sin(angle_rad)

# Try a range of launch angles
angles = np.radians(np.linspace(10, 80, 300))
hit_angle = None

for angle in angles:
    t = np.linspace(0, 5, num=500)
    x = v0 * np.cos(angle) * t
    y = v0 * np.sin(angle) * t - 0.5 * g * t**2

    # Collision check
    distance = np.sqrt((x - target_x)**2 + (y - target_y)**2)
    if np.any(distance < target_radius):
        hit_angle = angle
        break

# Plot result
if hit_angle is not None:
    t = np.linspace(0, 5, num=500)
    x = v0 * np.cos(hit_angle) * t
    y = v0 * np.sin(hit_angle) * t - 0.5 * g * t**2

    mask = y >= 0
    plt.plot(x[mask], y[mask], label=f"Hit Angle = {np.degrees(hit_angle):.2f}°")
    plt.plot(target_x, target_y, 'ro', label="Target")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.title("Projectile Trajectory")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()
else:
    print("No hit found for current initial speed.")
