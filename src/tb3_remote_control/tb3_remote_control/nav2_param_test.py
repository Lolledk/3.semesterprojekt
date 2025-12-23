import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


RS015 = pd.read_csv("RS015.csv")
SR015 = pd.read_csv("SR015.csv")
SR035 = pd.read_csv("SR035.csv")
RS035 = pd.read_csv("RS035.csv")
SR055 = pd.read_csv("SR055.csv")
RS055 = pd.read_csv("RS055.csv")
#plt.plot(RS015.x, RS015.y)
#plt.axis("equal")
#plt.xlabel("x [m]")
#plt.ylabel("y [m]")
#plt.title("Robot trajectory (map frame)")
#plt.show()



r = 52.5/(2*np.pi)/100
print("Radius:", r)
obstacle_x = (-2.708 + -2.473)/2
obstacle_y = (-0.087 + 2.057)/2
cx, cy = obstacle_x, obstacle_y
theta = np.linspace(0, 2*np.pi, 400)
x_c = cx + r * np.cos(theta)
y_c = cy + r * np.sin(theta)

def circle(r):
    theta = np.linspace(0, 2*np.pi, 400)
    x = obstacle_x + r * np.cos(theta)
    y = obstacle_y + r * np.sin(theta)
    return x, y

colors = {
    "SR015": "tab:blue",
    "SR035": "tab:orange",
    "SR055": "tab:green",
}

"""SR015: 39.4 s
RS015: 38.3 s

SR035: 30.6 s
RS035: 18.5 s

SR055: 164.6 s
RS055: 37.8 s

SR015_250: 23.0 s hedder RS
RS015_250: 27.1 s hedder SR"""

plt.figure(figsize=(8, 8))

# Trajectories
plt.plot(SR015.x, SR015.y, color=colors["SR015"], label="SR015, t = 39.4 s")
plt.plot(SR035.x, SR035.y, color=colors["SR035"], label="SR035, t = 30.6 s")
plt.plot(SR055.x, SR055.y, color=colors["SR055"], label="SR055, t = 164.6 s")

# Start / goal
plt.plot(-2.473, -0.087, 'ro', label="Point 1")
plt.plot(-2.708, 2.057, 'go', label="Point 2")

# Obstacle
plt.plot(circle(r)[0], circle(r)[1], '--', color='black',
         label=f"Obstacle (r={r:.2f} m)")

# Inflation circles (same color as corresponding trajectory)
plt.plot(circle(r + 0.15)[0], circle(r + 0.15)[1], ':',
         color=colors["SR015"], label="Inflation radius 15 cm")

plt.plot(circle(r + 0.35)[0], circle(r + 0.35)[1], ':',
         color=colors["SR035"], label="Inflation radius 35 cm")

plt.plot(circle(r + 0.55)[0], circle(r + 0.55)[1], ':',
         color=colors["SR055"], label="Inflation radius 55 cm")

plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory with obstacle from point 1 to point 2")
plt.legend()
plt.show()

colors = {
    "RS015": "tab:blue",
    "RS035": "tab:orange",
    "RS055": "tab:green",
}

plt.figure(figsize=(8, 8))

plt.plot(RS015.x, RS015.y, color=colors["RS015"], label="RS015, t = 38.3 s")
plt.plot(RS035.x, RS035.y, color=colors["RS035"], label="RS035, t = 18.5 s")
plt.plot(RS055.x, RS055.y, color=colors["RS055"], label="RS055, t = 37.8 s")

plt.plot(-2.473, -0.087, 'go', label="Point 1")
plt.plot(-2.708, 2.057, 'ro', label="Point 2")

plt.plot(x_c, y_c, '--', label=f"Obstacle (r={r:.2f} m)")
plt.plot(circle(r + 0.15)[0], circle(r + 0.15)[1], ':',
         color=colors["RS015"], label="Inflation radius 15 cm")
plt.plot(circle(r + 0.35)[0], circle(r + 0.35)[1], ':',
         color=colors["RS035"], label="Inflation radius 35 cm")
plt.plot(circle(r + 0.55)[0], circle(r + 0.55)[1], ':',
         color=colors["RS055"], label="Inflation radius 55 cm")

plt.axis("equal")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Trajectory with obstacle from point 2 to point 1")
plt.legend()
plt.show()


