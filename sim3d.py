'''
==============
Orbital simulation of a satellite
==============

Demonstration of a satellite's path around the earth in 3D.
'''

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np

plt.ion()
fig = plt.figure()

# ...............
import math

PI = 3.141592653589793
GM = 3.986005e14
R = 6378.14
M = 5.9737e24

class Satellite(object):
    """docstring for Satellite."""
    def __init__(self, mass, eccentricity, semi_major_axis, inclination, true_anomaly):
        super(Satellite, self).__init__()
        self.mass = mass
        self.eccentricity = eccentricity
        self.semi_major_axis = semi_major_axis
        self.inclination = inclination
        self.current_true_anomaly = true_anomaly
        self.elapsed_time = 0
        self.dt = 1

    def apogee_in_km(self):
        return self.semi_major_axis * (1+self.eccentricity)

    def perigee_in_km(self):
        return self.semi_major_axis * (1-self.eccentricity)

    def apogee_altitude_in_km(self):
        return self.apogee_in_km() - R

    def perigee_altitude_in_km(self):
        return self.perigee_in_km() - R

    def velocity_at_perigee(self):
        Ra = self.apogee_in_km() * 1000
        Rp = self.perigee_in_km() * 1000
        return math.sqrt(2 * GM * Ra / (Rp * (Ra + Rp)))

    def velocity_at_apogee(self):
        Ra = self.apogee_in_km() * 1000
        Rp = self.perigee_in_km() * 1000
        return math.sqrt(2 * GM * Rp / (Ra * (Ra + Rp)))

    def momentum_at_apogee(self):
        return (self.mass * self.velocity_at_apogee())

    def momentum_at_perigee(self):
        return (self.mass * self.velocity_at_perigee())

    def eccentric_anomaly(self):
        e = self.eccentricity
        mu = math.radians(self.current_true_anomaly)
        return math.acos((e + math.cos(mu)) / (1 + e * math.cos(mu))) # result in radians

    def mean_anomaly(self):
        e = self.eccentricity
        E = self.eccentric_anomaly()
        return E - e * math.sin(E) # result in radians

    def mean_motion(self):
        a = self.semi_major_axis * 1000
        return math.sqrt(GM / math.pow(a, 3))

    def calculate_current_true_anomaly(self):
        # mean anomaly given time step, dt = t - t0, initial angle for the true_anomaly and mean motion, n
        e = self.eccentricity
        Mo = self.mean_anomaly()
        n = self.mean_motion()
        M = Mo + n * (self.dt)
        E = M
        while True:
            dE = (E - e * math.sin(E) - M)/(1 - e * math.cos(E))
            E -= dE
            if abs(dE) < 1e-6:
                break
        mu = math.acos((math.cos(E) - e) / (1 - e * math.cos(E)))
        true_anomaly = math.degrees(mu)
        return true_anomaly
    
    def orbital_radius(self):
        e = self.eccentricity
        m = self.current_true_anomaly
        a = self.semi_major_axis
        r = a*(1-math.pow(e, 2))/1+(e*math.cos(m))
        return r
    
    def compute_current_postion(self):
        self.elapsed_time = self.elapsed_time + self.dt
        self.current_true_anomaly = self.calculate_current_true_anomaly()
        radius = self.orbital_radius()
        return [radius, self.current_true_anomaly, self.inclination, self.elapsed_time]

kuns = Satellite(1, 0.01, 7500, 30, 69) # Satellite(mass, eccentricity, semi_major_axis, inclination, true_anomaly)
t = 0
while t < 14400:
    data = kuns.compute_current_postion()
    r = data[0]
    v = data[1]
    i = data[2]
    et = data[3]
    x = r * math.sin(v) * math.cos(i)
    y = r * math.sin(v) * math.sin(i)
    z = r * math.cos(v)

    plt.clf()  # Clear the figure
    coordsX = np.array([x])
    coordsY = np.array([y])
    coordsZ = np.array([z])

    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(coordsX, coordsY, coordsZ, c='blue', marker='o')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    # Make data
    u = np.linspace(0, 2 * np.pi, 1000)
    v = np.linspace(0, np.pi, 1000)
    x = 6370 * np.outer(np.cos(u), np.sin(v))
    y = 6370 * np.outer(np.sin(u), np.sin(v))
    z = 7000 * np.outer(np.ones(np.size(u)), np.cos(v))

    # Plot the surface
    ax.plot_surface(x, y, z, color='b')

    plt.show()
    
    string = 'Time $t$ = '+str(et)+''
    plt.suptitle(string)
    # ax.axis('equal') 
    # ax.axis((-8000, 8000, -8000, 8000))
    ax.set_xlim3d(-8000, 8000)
    ax.set_ylim3d(-8000, 8000)
    ax.set_zlim3d(-8000, 8000)
    plt.pause(0.001)
