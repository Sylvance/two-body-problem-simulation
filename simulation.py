import math
import turtle

G = 6.67428e-11
M = 5.9742 * 10**24

class Body(object):
    def __init__(self, name, mass, velocity):
        self.name = name
        self.mass = mass
        self.velocity = velocity

    def __str__(self):
        return 'Name: {} Mass: {} Velocity: {}'.format(self.name, self.mass, self.velocity)
    
    def render_body(self, turtle_alias, params):
        turtle_alias.color(params['color'])
        turtle_alias.pensize(params['size'])
        turtle_alias.dot()


class CentralBody(Body):
    def __init__(self, name, mass, velocity):
        super(CentralBody, self).__init__(name, mass, velocity)

    def __str__(self):
        return super(CentralBody, self).__str__() + " [central mass]"

class OrbitingBody(Body):
    def __init__(self, name, mass, radius, velocity, mean_anomaly, eccentricity, semi_major_axis):
        super(OrbitingBody, self).__init__(name, mass, velocity)
        self.radius = radius
        self.mean_anomaly = mean_anomaly
        self.eccentricity = eccentricity
        self.semi_major_axis = semi_major_axis
    
    def current_positional_radius(self):
        e = self.eccentricity
        m = self.mean_anomaly
        a = self.semi_major_axis
        r = a*(1-math.pow(e, 2))/1+(e*math.cos(m))
        return r
    
    def current_velocity(self):
        r = self.current_positional_radius
        a = self.semi_major_axis
        kr = (2/r)-(1/a) 
        v = math.sqrt(G*M*kr)
        return v
    
    def move_body(self, turtle_alias, params):
        r = self.current_positional_radius()
        m = self.mean_anomaly
        turtle_alias.color(params['color'])
        turtle_alias.pensize(params['size'])
        turtle_alias.penup()
        turtle_alias.right(m)
        turtle_alias.forward(int(r))
        turtle_alias.pendown()
        turtle_alias.dot()
        turtle_alias.color('white')
        turtle_alias.dot()
        print("Moved to {}".format(turtle_alias.pos()))
        return True
    
    def __str__(self):
        return super(OrbitingBody, self).__str__() + " [orbiting mass]"

def loop(earth, satellite):
    """Loop through the values to animate"""
    time = 0 # Initialize time
    timestep = 60 # One minute
    timelimit = 3600 # One day
    primary = turtle.Turtle()
    secondary = turtle.Turtle()
    secondary.speed(0)
    primary_params = {
        'color': 'red',
        'size': 25
    }
    secondary_params = {
        'color': 'yellow',
        'size': 12
    }
    earth.render_body(primary, primary_params)
    while time < timelimit:
        satellite.move_body(secondary, secondary_params)
        time+= timestep
    return timestep

def main():
    earth = CentralBody('earth', M, 0)
    # satellite(name, mass, radius, velocity, mean_anomaly, eccentricity, semi_major_axis)
    satellite = OrbitingBody('satellite', 10, 62.3, 29.3, 53, 0.6, 200)
    wn = turtle.Screen()
    loop(earth, satellite)
    wn.exitonclick()

if __name__ == '__main__':
    main()
