""" main factory maker algorithm  """

import json
import logging

# importing libraries/class
from optimizer import FloorOptimizer

# constants
FILE_PATH_INPUT = 'input_dir/'
FILE_PATH_OUTPUT = "output_dir/"

if __name__ == '__main__':

    try:
        # parsing json input_dir file
        with open(FILE_PATH_INPUT + 'plan_template.json', 'r') as file:
            # fetching data from json file
            data = json.load(file)

        # Note : adding rotation to the rectangle and path can improve results and relax constraints
        # running optimization algorithm (adjust params: penalty and iterations to get varied results )
        opt = FloorOptimizer(data, FILE_PATH_OUTPUT, max_iteration=500, penality_out_of_bounds=1e6,
                             penality_min_area=1e4,
                             penality_overlapping_rect=1e6,
                             penality_path_not_connecting_rect=1e8)
        opt.trigger_optimization()

    except Exception as e:
        logging.getLogger().setLevel(logging.DEBUG)
        print("------- Computation failed ------- ", "[ Reason: " + str(e) + "]")

