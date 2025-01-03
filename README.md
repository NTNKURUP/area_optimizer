
# CONTENTS

This directory contains the python codes for factory floor optimization

The main file to run is :
1. main.py

## Algorithm flow:
1. Run the main.py file once all necessary libraries are installed
2. The main.py file flow
    * Parse the json input file available in input_dir
    * run the main function  FloorOptimizer(data, contraints). This function will trigger the optimization. Adjusting the constraints will lead to varied outputs
    * Final plot of result and the json output from optimization will be stored in the output_dir
        
    Current output limitations: Path optimization still requires work to connect 2 buildings, 
    constraint adjustment and adding rotation as an input variable for optimization needed.

## Methodology:
1. The optimizer starts with computing the total area to minimize ( sum of area of all rectangles), and this value was multiplies (1.5 times) to allow the optimizer more freedom to search (Assumsion).
    Then the input parameters are set, which are scaled and normalized values of the dimensions between (0 -1), which is easier for search.
   These scaled value is then used to calculate total area( mentioned above), along with the initial reference points for each rectangles in the plane.
   For ease of understanding: I have considered all the paramerters as rectangles, the building considered as fixed dimension rectangle whose refernce point only need to optimized and the path are considered as rectangles with fixed widht and varying height/lenght.
   for the varying rectangles the position and height variable need to be optimized.
    The upper and lower bounds of the parameter ( reference points for all rectangles and height for varying rectangles) were set.
   These variables were then passed into differential_evolution optimizer. 
   optimization function : miminze total area covered by rectangles + minimize (height or lenght) of varying rectangles
   constraints added were: going out of bounds ( rectangles tend to move out of the max computed area) were penalzed
                            overlapping rectangles were penalized
                            penalities were added if rectangles were not minimizing area and 
                            penality for not connecting 2 unique rectangles with varying lenght rectangle 
   Note: constraint was not added to limit the path connection to specific building pairs, rather connection between unique pair of rectangles was added.
   A call back function was added to get the best local solution in case of premature closing of optimizer. the result are then un normalized and multiplied with scale factor and returned as result.
    1. varying the constraints highly influences the result.
    2. adding additional control points, such as rotation to rectangles (fixed and varying).
    3. initial control points also highly influnece the optimization
    4. size of rectangles to fit within area and not able to rotate the path causes issue.
                        
2. Some of the results , trial runs (collection) can be seen in the results_collection folder

### INSTALLATION DEPENDENCIES

To install python dependencies use the command in pycharm: python -m pip install -r requirements.txt


The algorithms should work from Python version 3.8. Below are the dependencies with 
versions, on which the algorithm was tested: main libraries to install
* matplotlib==3.7.5
* numpy==1.24.4
* scipy==1.10.1
* shapely==2.0.6
 

### META                                                                                                                
*Updated on*  : 02-01-2025             
*Repo-link*  : https://github.com/NTNKURUP/area_optimizer/tree/main
