from __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import math
import csv

demand = [20,20,20,20,20,20,20,20,20,20]
distance = [
    [0, 26, 23, 29, 22, 27, 28, 28, 27, 22, 25],
    [24, 0, 4, 5, 10, 13, 12, 5, 10, 8, 6],
    [22, 4, 0, 10, 8, 11, 12, 9, 9, 6, 7],
    [27, 4, 8, 0, 10, 10, 8, 2, 6, 9, 3],
    [22, 11, 8, 12, 0, 8, 8, 11, 7, 3, 9],
    [27, 13, 12, 10, 8, 0, 2, 9, 3, 9, 8],
    [28, 11, 12, 9, 9, 2, 0, 8, 3, 9, 6],
    [28, 5, 8, 2, 10, 10, 8, 0, 6, 10, 3],
    [28, 9, 10, 7, 7, 4, 3, 6, 0, 6, 4],
    [23, 9, 6, 11, 3, 8, 8, 10, 6, 0, 7],
    [25, 6, 6, 4, 8, 8, 7, 3, 5, 7, 0],
]
dict_inx = {}
prev = 0
for i,d in enumerate(demand):
    # print(prev)
    dict_inx.update({i+1:(list(range(prev+1,prev+1+math.floor(d/2))))})
    prev += math.floor(d/2)
Manipulated_Demand =[]
Manipulates_Distance = []
for i in demand:
    if i%2 == 0:
        Manipulated_Demand.extend([2]*math.floor(i/2))
    else:
        Manipulated_Demand.extend([2]*(math.floor(i/2)-1))
        Manipulated_Demand.append(3)
fc_row = [0]
for i in range(len(demand)):
    temp = [0]
    for j in range(len(demand)):
        temp.append(distance[i+1][j+1])
        x = temp[-1]
        temp.extend([x] * (math.floor(demand[i] / 2)-1))
    for k in range(math.floor(demand[i] / 2)):
        Manipulates_Distance.append(temp)
    fc_row.append(distance[0][i+1])
    x = fc_row[-1]
    fc_row.extend([x] * (math.floor(demand[i] / 2)-1))
Manipulates_Distance.insert(0,fc_row)
Manipulated_Demand.insert(0,0)
print(dict_inx)
def search_node(val):
    for node, node_list in dict_inx.items():
        if val in node_list:
            return node
def create_data_model():
    data = {}
    data['distance_matrix'] = Manipulates_Distance
    data['num_vehicles'] = 50
    data['demands'] = Manipulated_Demand
    data['vehicle_capacities'] = [28]*50
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, assignment):
    total_distance = 0
    total_load = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        node = 0
        last = 0
        last_sum = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            node += 1
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(search_node(node_index), route_load)
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            # print(previous_index,index)
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
            print("....", route_distance, routing.GetArcCostForVehicle(previous_index, index, vehicle_id))
        plan_output += ' {0} Load({1})\n'.format(search_node(manager.IndexToNode(index)),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        if node > 1:
            print(plan_output)
        total_distance += route_distance
        total_load += route_load

def main():
    data = create_data_model()
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)


    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)


    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]
    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    # Allow to drop nodes.
    # penalty = 10
    # for node in range(1, 7):
    #     routing.AddDisjunction([manager.NodeToIndex(node)], penalty)

    plus_one_callback_index = routing.RegisterUnaryTransitCallback(lambda index :1)
    count_dimension_name='count'
    routing.AddDimension(
        plus_one_callback_index,
        0,
        20,
        True,
        count_dimension_name
    )
    count_dimension=routing.GetDimensionOrDie(count_dimension_name)


    for vehicle_idx in range (0,data['num_vehicles']):
        index=routing.End(vehicle_idx)
        count_dimension.CumulVar(index).SetRange(0,20)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        print_solution(data, manager, routing, assignment)


if __name__ == '__main__':
    for i in range(100):
    main()
