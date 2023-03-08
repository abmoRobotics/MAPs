# from cuopt import routing
# from cuopt import distance_engine
# import cudf
# from scipy.spatial import distance
# import numpy as np
# import requests
# import polyline
# import folium
# import json
# import pandas as pd
# from cuopt.routing import utils


# number_of_shuttles = 5
# number_of_manipulators = 5
# Shuttle_Cap = np.ones(number_of_shuttles)
# Manipulator_Cap = np.ones(number_of_manipulators)
# Vehicle = [Shuttle_Cap,Manipulator_Cap]
# start_location = [[1,1],[1,2][1,3][1,4][1,5][1,6][1,7][1,8][1,9][1,10]]
# location_names = []

# How muche each location weight
# location_demand = [1,2,4,5,6,7,8,3,9,12]

# vehicle_capacity = [1,1,1,1,1,1,1,1,1,1]

# n_locations = len(location_demand)
# n_vehicles = len(vehicle_capacity)


# for i in len(Shuttle_Cap): location_names.append("Shuttle"+str(i)) 
# print(location_names)

# location_coordinates_df = pd.DataFrame(start_location, columns=['xcord', 'ycord'], index=location_names)




# points = cudf.DataFrame({"x_coord": [1, 1, 2, 3], "y_coord":[3, 1, 4, 1]})
# cost_matrix = distance.cdist(points.to_pandas().values, points.to_pandas().values, "euclidean")

# distance_matrix  = cudf.DataFrame(cost_matrix, 
#                                   index=location_coordinates_df.index, 
#                                   columns=location_coordinates_df.index)


# data_model = routing.DataModel(n_locations, n_vehicles)

# set the cost matrix
# data_model.add_cost_matrix(distance_matrix)

# add a capacity dimension for the deliveries
# data_model.add_capacity_dimension(
#     "deliveries",
#     cudf.Series(location_demand),
#     cudf.Series(vehicle_capacity)
# )

# solver_settings = routing.SolverSettings()

# set number of climbers that will try to search for an optimal routes in parallel
# solver_settings.set_number_of_climbers(128)

# solver_settings will run for given time limit.  Larger and/or more complex problems may require more time.
# solver_settings.set_time_limit(0.01)

# routing_solution = routing.Solve(data_model, solver_settings)
# if routing_solution.get_status() == 0:
#     print("Cost for the routing in distance: ", routing_solution.final_cost)
#     print("Vehicle count to complete routing: ", routing_solution.vehicle_count)
#     utils.show_vehicle_routes(routing_solution.route, location_names)
#     routing_solution.route
# else:
#     print("NVIDIA cuOpt Failed to find a solution with status : ", routing_solution.get_status())