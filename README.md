# OskarsCube
This is a project done in the 3d-printing course in TAU, which creates a general 3D Oskar's Cube, with the solution in Python.

## How-To-Use:
#### Assertions and Requirements:
* First assure that the .py file is in the same folder containing the "input" folder with cube.stl.
* [numpy](http://www.numpy.org/) - Required for numpy-stl
* [python-utils](https://pypi.python.org/pypi/python-utils) - Required for numpy-stl
* [numpy-stl](https://github.com/WoLpH/numpy-stl) - STL manipulation framework




#### Arguments
The arguments of the python script are the sizes
of the maze over the three axes - X,Y,Z, and then the minimal
length of the solution path, separated by spaces.
The minimal length is optional, and set as 0 if not given.

Therefore the following are some examples of calling the code:
#### Examples
Generate a maze with size 2 on X, 3 on Y, 4 on Z and solution length of at least 7:
python mazeGen3d.py 2 3 4 7

Generate a maze with size 2 on X, 3 on Y, 4 on Z and solution length of at least 1000 (This will fail as no such maze exists):
python mazeGen3d.py 2 3 4 1000

Generate a maze with size 5 on X, 5 on Y, 5 on Z and without constraint on solution length:
python mazeGen3d.py 5 5 5

#### Output
The outputs are in the "output" folder located in the folder containing the .py file.
