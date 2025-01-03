import json
import logging
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import differential_evolution
from shapely.geometry import Polygon, LineString


class FloorOptimizer:
    """
    Class for factory space optimization
    Args:
    param1: form building data parsed from UI
    param2: Path
    Optional: Additional info (list of dict) for passing extra data"""

    def __init__(self, building, path, **kwargs):
        # miscellaneous info
        self.building_params = building
        self.out_dir_path = path
        self.additional_info = kwargs
        self.maxiter = self.additional_info['max_iteration']
        self.penality_out_of_bounds = self.additional_info['penality_out_of_bounds']
        self.penality_min_area = self.additional_info['penality_min_area']
        self.penality_overlapping_rect = self.additional_info['penality_overlapping_rect']
        self.penality_path_not_connecting_rect = self.additional_info['penality_path_not_connecting_rect']

        self.total_area = None
        self.bounds = None
        self.initial_values = None
        self.result = None
        self.best_solution = None  # for printing out the best results
        self.best_value = np.inf
        self.scaling_factor = None
        self.rectangles = self.set_params()

    def trigger_optimization(self):
        """ main optimization function """
        try:
            self.compute_total_area()
            self.get_normalized_control_points()
            self.set_bounds_for_params()
            self.run_optimizer()
            self.save_plot_result(False)

        except Exception as e:
            logging.getLogger().setLevel(logging.DEBUG)
            logging.error(str(e), exc_info=True)
            print(e)

    def set_params(self):  # hardcoded to accept only 4 rect and 2 variable path
        """ rearrarnging parsed data and applying scaling factor """
        max_dimension = max(
            max(building['dimensions']) for building in self.building_params['buildings']
        )
        max_dimension = max(max_dimension, max(path['max_length'] for path in self.building_params['paths']))
        self.scaling_factor = max_dimension  # Save the scaling factor

        rectangles = []
        # Add buildings as fixed rectangles
        for building in self.building_params['buildings']:
            rectangles.append({
                'name': building['name'],
                'width': building['dimensions'][0] / self.scaling_factor,
                'height': building['dimensions'][1] / self.scaling_factor
            })

        # Add paths as variable rectangles
        for path in self.building_params['paths']:
            rectangles.append({
                'name': path['name'],
                'width': path['width'] / self.scaling_factor,
                'min_height': path['min_length'] / self.scaling_factor,
                'max_height': path['max_length'] / self.scaling_factor
            })
        return rectangles

    def compute_total_area(self):
        """ computing initial total area of rectangles for target area to minimize during optimization ( mulitply 1.5) """
        # Calculate the total area area * 1.5 times
        total_area = sum(
            [rect['width'] * rect['height'] if 'height' in rect else rect['width'] * rect['max_height'] for rect in
             self.rectangles]) * 1.5
        self.total_area = total_area
        print("the initial bounding area is:", self.total_area)

    def get_normalized_control_points(self):
        """ random initial control points of positions and height ( or length) of rectangles """
        # Define initial control points
        initial_positions = np.array(
            [[0.1, 0.2], [0.3, 0.5], [0.2, 0.3], [0.3, 0.1], [0.1, 0.4],
             [0.2, 0.1]])  # TODO hardcoding these inputs for 6 rectangles no time to fix
        initial_heights = np.array([self.rectangles[-2]['min_height'], self.rectangles[-1]['min_height']])
        self.initial_values = np.concatenate([initial_positions.flatten() / self.total_area, initial_heights])
        # Ensure all initial values are within the range [0, 1]
        # self.initial_values = np.clip(initial_values, 0, 1)

    # Helper function to create a non-rotated rectangle
    def create_rectangle(self, x, y, width, height):
        """ function to draw a rectangle """
        corners = [(x, y), (x + width, y), (x + width, y + height), (x, y + height)]
        return Polygon(corners)

    def callback(self, xk, convergence):
        """callback function to store best result in case of premature termination of optimizer """
        current_value, bounding_area, penalty, variable_heights = self.objective_with_penalty(xk)
        print(
            f"Iteration: Objective value: {current_value}, Bounding Area: {bounding_area}, Penalty: {penalty}, Variable Heights: {variable_heights}")

        if current_value < self.best_value:
            self.best_solution = xk
            self.best_value = current_value
            print(f"New best value: {self.best_value} at positions {xk}")

            # Objective function: minimize the bounding area and length of varing rectangles

    def objective_with_penalty(self, values):
        """main objective function: minimze total rect bounding area and height/lenght of rectangles added with constraints """
        positions = values[:len(self.rectangles) * 2].reshape(-1, 2) * self.total_area
        variable_heights = [
            values[-2] * (self.rectangles[-2]['max_height'] - self.rectangles[-2]['min_height']) + self.rectangles[-2][
                'min_height'],
            values[-1] * (self.rectangles[-1]['max_height'] - self.rectangles[-1]['min_height']) + self.rectangles[-1][
                'min_height']
        ]

        x_coords = []
        y_coords = []
        penalty = 0

        polygons = []
        # Add rectangles
        for i, rect in enumerate(self.rectangles):
            x = positions[i, 0]
            y = positions[i, 1]
            if i < len(self.rectangles) - 2:
                height = rect['height']
            else:
                height = variable_heights[i - len(self.rectangles) + 2]

            width = rect['width']
            polygon = self.create_rectangle(x, y, width, height)
            polygons.append(polygon)
            x_coords.extend(polygon.exterior.xy[0])
            y_coords.extend(polygon.exterior.xy[1])

            # Add increased penalty for rectangles being out of bounds
            min_x, min_y, max_x, max_y = polygon.bounds
            if min_x < 0 or max_x > self.total_area or min_y < 0 or max_y > self.total_area:
                penalty += self.penality_out_of_bounds  # 1e6

        bounding_area = (max(x_coords) - min(x_coords)) * (max(y_coords) - min(y_coords))

        # Check if the bounding area is not less than the room area
        if bounding_area > self.total_area:
            penalty += self.penality_min_area  # Example penalty value 1e4

        # Add penalty for overlapping rectangles
        for i in range(len(polygons)):
            for j in range(i + 1, len(polygons)):
                if polygons[i].intersects(polygons[j]):
                    penalty += self.penality_overlapping_rect  # Extremely high penalty for overlapping rectangles 1e6

            # Find the two closest rectangles for each variable rectangle and ensure unique paths
            distances = []
            used_indices = set()
            for i in range(len(self.rectangles) - 2):
                for j in range(i + 1, len(self.rectangles) - 2):
                    poly1 = polygons[i]
                    poly2 = polygons[j]
                    distances.append((poly1.distance(poly2), i, j))
            distances.sort()

            closest_rects = []
            for dist, i, j in distances:
                if i not in used_indices and j not in used_indices:
                    closest_rects.append((i, j))
                    used_indices.add(i)
                    used_indices.add(j)
                if len(closest_rects) == 2:
                    break

            for idx, rect_idx in enumerate(closest_rects):
                rect1 = polygons[rect_idx[0]]
                rect2 = polygons[rect_idx[1]]
                line = LineString([rect1.centroid, rect2.centroid])

                mid_point = line.interpolate(0.5, normalized=True)
                positions[-4 + idx * 2] = mid_point.x / self.total_area  # Normalize the position
                positions[-3 + idx * 2] = mid_point.y / self.total_area  # Normalize the position

                if not line.intersects(polygons[-2 + idx]):
                    penalty += self.penality_path_not_connecting_rect  # 1e5 Add penalty if the variable rectangle doesn't connect the two closest rectangles

        # Objective value includes the bounding area, penalty, and the height of the connecting rectangles
        objective_value = bounding_area + penalty + sum(variable_heights)
        return objective_value, bounding_area, penalty, variable_heights

    def set_bounds_for_params(self):
        """setting upper and lower bounds for the optimizer """
        self.bounds = [(0, 1) for _ in range(len(self.rectangles) * 2)] + [(0, 1) for _ in range(2)]

    # Run differential evolution optimization with increased iterations
    def run_optimizer(self):
        """main optimizer function """
        print("Starting Optimization")
        self.result = differential_evolution(lambda x: self.objective_with_penalty(x)[0], self.bounds,
                                             maxiter=self.maxiter, callback=self.callback,
                                             init='latinhypercube', x0=[self.initial_values])
        # Output results and plot
        if self.result.success:
            print("Optimization successful!")
        else:
            print("Optimization failed. Returning best found solution.")
            self.result.x = self.best_solution  # Use the best solution found so far

    def denormalize_results(self):
        # Denormalize positions
        positions = self.result.x[:len(self.rectangles) * 2].reshape(-1, 2) * self.total_area * self.scaling_factor

        # Denormalize variable heights
        variable_heights = [
            (self.result.x[-2] * (self.rectangles[-2]['max_height'] - self.rectangles[-2]['min_height']) +
             self.rectangles[-2]['min_height']) * self.scaling_factor,
            (self.result.x[-1] * (self.rectangles[-1]['max_height'] - self.rectangles[-1]['min_height']) +
             self.rectangles[-1]['min_height']) * self.scaling_factor
        ]
        # calculating bounding area
        # Calculate bounding area
        min_x = min(positions[:, 0])
        max_x = max(positions[:, 0])
        min_y = min(positions[:, 1])
        max_y = max(positions[:, 1])
        bounding_area = (max_x - min_x) * (max_y - min_y)

        return positions, variable_heights, bounding_area

    def save_plot_result(self, is_show_plot=False):
        """function to save png image of plot and json file output in output directory """
        # Denormalize positions and variable heights
        positions, variable_heights, bounding_area = self.denormalize_results()
        print("positions", positions)

        # Plot the optimized arrangement
        fig, ax = plt.subplots()

        # Set plot limits based on scaled total area
        ax.set_xlim(0, self.total_area * self.scaling_factor)
        ax.set_ylim(0, self.total_area * self.scaling_factor)

        # Plot each rectangle
        for i, rect in enumerate(self.rectangles):
            x, y = positions[i]
            if i < len(self.rectangles) - 2:
                height = rect['height'] * self.scaling_factor
                color = 'lightblue'
            else:
                height = variable_heights[i - len(self.rectangles) + 2] * self.scaling_factor
                color = 'brown'

            width = rect['width'] * self.scaling_factor
            polygon = self.create_rectangle(x, y, width, height)
            x_poly, y_poly = polygon.exterior.xy
            ax.plot(x_poly, y_poly, color='black')
            ax.fill(x_poly, y_poly, facecolor=color, edgecolor='black', alpha=0.5)
            ax.text(np.mean(x_poly), np.mean(y_poly), rect['name'], ha='center', va='center')

        ax.set_xlim(0, self.total_area * self.scaling_factor)
        ax.set_ylim(0, self.total_area * self.scaling_factor)
        ax.set_aspect('equal')
        ax.set_title('Optimized arrangement')
        ax.set_xlabel(f'X-axis (1.5 * max area of rect combined)')
        ax.set_ylabel(f'Y-axis (1.5 * max area of rect combined)')

        # Save the plot to a directory as an image
        plt.savefig(self.out_dir_path + 'optimized_graph.png')

        if is_show_plot:
            plt.show()
        else:
            plt.close(fig)  # Close the figure to avoid display in non-interactive environments

        # Exporting json result  # TODO make a separate function
        buildings = []
        for i, rect in enumerate(self.rectangles):
            name = rect['name']
            x, y = positions[i]
            width = rect['width'] * self.scaling_factor
            height = rect['height'] * self.scaling_factor if 'height' in rect else variable_heights[
                i - len(self.rectangles) + 2]
            dimensions = [width, height]
            buildings.append({"name": name, "location": [x, y], "dimensions": dimensions})

        paths = []
        used_pairs = set()
        distances = []

        for i in range(len(self.rectangles) - 2):
            for j in range(i + 1, len(self.rectangles) - 2):
                distance = np.linalg.norm(positions[i] - positions[j])
                distances.append((distance, i, j))
        distances.sort()

        path_id = 1
        for distance, i, j in distances:
            if (i, j) not in used_pairs and (j, i) not in used_pairs:
                used_pairs.add((i, j))
                path_name = f"Path {path_id}"
                connected_buildings = [self.rectangles[i]['name'], self.rectangles[j]['name']]
                height_minimized = min(positions[i][1] + variable_heights[0], positions[j][1] + variable_heights[1])
                paths.append({"name": path_name, "length": distance, "connected_buildings": connected_buildings,
                              "height_minimized": height_minimized})
                path_id += 1
                if len(paths) >= 2:  # Assuming two paths to connect two variable rectangles
                    break

        arrangement_result = {
            "arrangement_id": 1,
            "buildings": buildings,
            "paths": paths,
            "total_area": bounding_area,
            "objectives": {"total_height_minimized": sum(variable_heights), "total_area": bounding_area}
        }

        with open(self.out_dir_path + 'arrangement_result.json', 'w') as outfile:
            json.dump(arrangement_result, outfile, indent=4)

        print("Result exported successfully!")
