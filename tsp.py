#!/usr/bin/env python3

import constants
import csv
import ortools.constraint_solver
import ortools.constraint_solver.pywrapcp
import requests
import sys

# https://developers.google.com/optimization/routing/tsp
def optimize_routes(distance_matrix, depot):

    manager = ortools.constraint_solver.pywrapcp.RoutingIndexManager(len(distance_matrix), 1, depot) # use 1 vehicle
    routing = ortools.constraint_solver.pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return distance_matrix[from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Setting first solution heuristic.
    search_parameters = ortools.constraint_solver.pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        ortools.constraint_solver.routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    solution = routing.SolveWithParameters(search_parameters)

    return build_path(manager, routing, solution)

    # if solution:
    #     print_solution(manager, routing, solution)

    # return solution

def build_path(manager, routing, solution):
    path = []
    index = routing.Start(0)
    while not routing.IsEnd(index):
        path.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))

    return path

def print_solution(manager, routing, solution):
    # print('Objective: {}'.format(solution.ObjectiveValue()))
    index = routing.Start(0)
    plan_output = "Route for vehicle:\n"
    route_distance = 0
    while not routing.IsEnd(index):
        plan_output += " {} ->".format(manager.IndexToNode(index))
        previous_index = index
        index = solution.Value(routing.NextVar(index))
        route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
    plan_output += " {}\n".format(manager.IndexToNode(index))
    # plan_output += 'Route distance: {}\n'.format(route_distance)
    print(plan_output)


def send_request(origins, destinations, api_key):
    url = "https://maps.googleapis.com/maps/api/distancematrix/json"
    params = {
        "units": "metric",
        "origins": "|".join(origins),
        "destinations": "|".join(destinations),
        "key": api_key
    }
    return requests.get(url, params).json()


# https://developers.google.com/optimization/routing/vrp#distance_matrix_api
def create_distance_matrix(addresses, api_key):

    def build_distance_matrix(response):
        return [[element["distance"]["value"] for element in row["elements"]] for row in response["rows"]]

    # Distance Matrix API only accepts 100 elements per request, so get rows in multiple requests.
    max_elements = 100
    num_addresses = len(addresses)
    # Maximum number of rows that can be computed per request
    max_rows = max_elements // num_addresses
    # num_addresses = q * max_rows + r
    q, r = divmod(num_addresses, max_rows)
    dest_addresses = addresses
    distance_matrix = []
    # Send q requests, returning max_rows rows per request.
    for i in range(q):
        origin_addresses = addresses[i * max_rows: (i + 1) * max_rows]
        response = send_request(origin_addresses, dest_addresses, api_key)
        distance_matrix += build_distance_matrix(response)

    # Get the remaining remaining r rows, if necessary.
    if r > 0:
        origin_addresses = addresses[q * max_rows: q * max_rows + r]
        response = send_request(origin_addresses, dest_addresses, api_key)
        distance_matrix += build_distance_matrix(response)

    return distance_matrix




def parse_input(filename):
    with open(filename) as csvfile:
        paradas = csv.DictReader(csvfile, delimiter=',', quotechar='|')
        return list(paradas)


def coords_string(latlong_dict):
    return ",".join([latlong_dict["lat"], latlong_dict["long"]])


def build_itinerary_url(input, order):
  url = "".join([
      "https://www.google.com/maps/dir/",
      "/".join([coords_string(input[i]) for i in order]),
      "/@",
      coords_string(input[order[0]])
  ])
  return url


def main(input_filename):
    input = parse_input(input_filename)
    # addresses = [addr["direccion"].replace(" ", "+") for addr in input]
    addresses = [coords_string(addr) for addr in input]
    matrix = create_distance_matrix(addresses, constants.api_key)
    results = optimize_routes(matrix, 0)

    print("Itinerario:")
    for i in range(len(results)):
        print(i, input[results[i]]["direccion"])

    print()
    print(build_itinerary_url(input, results))


if __name__ == "__main__":
    main(sys.argv[1])
