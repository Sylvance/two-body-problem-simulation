# Two body problem simulation
Simulation of the classical two-body problem in the context of the earth and artificial satellites.

## Context of this project
- This program was made as a result of my fourth year project work in Astronomy.

## Study
- [Orbital mechanics](http://www.braeunig.us/space/orbmech.htm)
- [TLE](http://www.stltracker.com/resources/tle)

## Simulation Equations
- Considering a bodies in such condtions below;
![Image1](http://www.braeunig.us/space/pics/fig4-03.gif)
- Then we are able to define `R` and `V`.

### R, the position from the primary body of the orbiting body
![Image2](http://www.braeunig.us/space/pics/eq4-43.gif)

### V, the orbiting body's velocity
![Image3](http://www.braeunig.us/space/pics/eq4-45.gif)

## Dependecies
- numpy
- matplotlib
- pandas

## Running with local python
### Installation
- Install dependencies with python-3 `pip3 install numpy matplotlib pandas`

### Running the simulation
- Run with python-3 `python3 sim3d_animated.py`

## Running with Docker
- Run `docker run sylvance/kuns-orbit:latest`

## Running with Docker-compose
- Run `docker up`

## Acknowledgements
- Travis pipeline - https://medium.com/mobileforgood/patterns-for-continuous-integration-with-docker-on-travis-ci-71857fff14c5

## Author
Sylvance Mbaka
