from __future__ import print_function

import math
import random
from typing import Callable

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import csv

# demand = [20, 20, 25, 20, 30, 20, 20, 20, 20, 20, ]
result = []
distance = [
    [0, 26, 23, 29, 22, 27, 28, 28, 27, 22, 25], #0
    [24, 0, 4, 5, 10, 13, 12, 5, 10, 8, 6], #1
    [22, 4, 0, 10, 8, 11, 12, 9, 9, 6, 7],  #2
    [27, 4, 8, 0, 10, 10, 8, 2, 6, 9, 3],   #3
    [22, 11, 8, 12, 0, 8, 8, 11, 7, 3, 9],  #4
    [27, 13, 12, 10, 8, 0, 2, 9, 3, 9, 8],  #5
    [28, 11, 12, 9, 9, 2, 0, 8, 3, 9, 6],   #6
    [28, 5, 8, 2, 10, 10, 8, 0, 6, 10, 3],  #7
    [28, 9, 10, 7, 7, 4, 3, 6, 0, 6, 4],    #8
    [23, 9, 6, 11, 3, 8, 8, 10, 6, 0, 7],   #9
    [25, 6, 6, 4, 8, 8, 7, 3, 5, 7, 0],     #10
]


def Manipulation(demand, distance):
    updated = {}
    Manipulated_Demand = []
    Manipulates_Distance = []
    for i in demand:
        if i % 2 == 0:
            Manipulated_Demand.extend([2] * math.floor(i / 2))
        else:
            Manipulated_Demand.extend([2] * (math.floor(i / 2) - 1))
            Manipulated_Demand.append(3)
    Manipulated_Demand.insert(0, 0)
    updated["Demand"] = Manipulated_Demand
    dict_inx = {}
    prev = 0
    for i, d in enumerate(demand):
        dict_inx.update({i + 1: (list(range(prev + 1, prev + 1 + math.floor(d / 2))))})
        prev += math.floor(d / 2)
    updated["Parent_MDC"] = dict_inx
    for i in range(1, len(Manipulated_Demand)):
        temp = [0]
        row = search_node(i, dict_inx)
        for j in range(1, len(Manipulated_Demand)):
            col = search_node(j, dict_inx)
            temp.append(distance[row][col])
        Manipulates_Distance.append(temp)

    temp = [0]
    for i in range(1, len(Manipulated_Demand)):
        col = search_node(i, dict_inx)
        temp.append(distance[0][col])
    Manipulates_Distance.insert(0, temp)
    updated["Distance"] = Manipulates_Distance
    updated["Parent_MDC"] = dict_inx
    return updated


def search_node(val, dict_inx):
    for node, node_list in dict_inx.items():
        if val in node_list:
            return node


'''
def Distance_Manipulation(distance, demand):

    Manipulates_Distance =[]
    print(demand)
    temp = [0]
    for i in range(len(demand)):
        temp = []
        for j,d in enumerate(demand):
            for k in range(math.floor(i / 2)):
                temp.append(distance[i+1][j+1])
        print(temp)
        for k in range(math.floor(i / 2)):
            Manipulates_Distance.append(temp)
    print(Manipulates_Distance)
    # return  Manipulates_Distance

fc_row = [0]
for i in range(len(demand)):
    temp = [0]
    for j in range(1,len(demand)):
        temp.append(distance[i][j])
        x = temp[-1]
        temp.extend([x] * (math.floor(demand[i] / 2) - 1))
    for k in range(math.floor(demand[i] / 2)):
        Manipulates_Distance.append(temp)
    fc_row.append(distance[0][i + 1])
    x = fc_row[-1]
    fc_row.extend([x] * (math.floor(demand[i] / 2) - 1))
Manipulates_Distance.insert(0, fc_row)
Manipulated_Demand.insert(0, 0)
print(Manipulated_Demand)
print(len(Manipulated_Demand))
for i in Manipulates_Distance:
    print(len(i),",,,,,",i)

def search_node(val):
    for node, node_list in dict_inx.items():
        if val in node_list:
            return node'''


def create_data_model(updated_data):
    data = {}
    data['distance_matrix'] = updated_data['Distance']
    data['num_vehicles'] = 15
    data['demands'] = updated_data["Demand"]
    data['vehicle_capacities'] = [28] * 15
    data['depot'] = 0
    data["cost_per_demand_unit"] = 0
    data["vehicle_VarCost"] = [10] * 15
    data['Fixed_cost'] = [1000] * 15
    return data


def print_solution(data, manager, routing, assignment, dic):
    total_distance = 0
    total_load = 0
    total_vehicle = 0
    t = []
    one = 0
    two = 0
    three = 0
    more_than_three = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        node = 0
        count_same = 1
        route = []
        while not routing.IsEnd(index):
            temp = []
            node_index = manager.IndexToNode(index)
            node += 1
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(search_node(node_index, dic), route_load)
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            if search_node(previous_index, dic) == search_node(index, dic) and previous_index is not None:
                count_same += 1
            elif previous_index is not None:
                temp.append(search_node(previous_index, dic))
                temp.append(count_same)
                route.append(temp)
                count_same = 1
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(search_node(manager.IndexToNode(index), dic),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        if len(route) == 2:
            one += 1
        elif len(route) == 3:
            two += 1
        elif len(route) == 4:
            three += 1
        elif len(route) > 4:
            more_than_three += 1
        if node > 1:
            print(plan_output)
            total_vehicle += 1
        total_distance += route_distance
        total_load += route_load
    result.append(total_distance)
    result.append(total_vehicle)
    result.append(one)
    result.append(two)
    result.append(three)
    result.append(more_than_three)
    print(result)


def main(updated_data):
    data = create_data_model(updated_data)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    routing = pywrapcp.RoutingModel(manager)

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

    evaluator_list = []

    def create_cost_callback(data, var, f):
        """Created call back to get total cost between locations """
        print('inside')
        l = []
        temp_cost = 0from __future__ import print_function

import math
import random
from typing import Callable

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2
import csv

# demand = [20, 20, 25, 20, 30, 20, 20, 20, 20, 20, ]
result = []
distance = [
    [0, 26, 23, 29, 22, 27, 28, 28, 27, 22, 25], #0
    [24, 0, 4, 5, 10, 13, 12, 5, 10, 8, 6], #1
    [22, 4, 0, 10, 8, 11, 12, 9, 9, 6, 7],  #2
    [27, 4, 8, 0, 10, 10, 8, 2, 6, 9, 3],   #3
    [22, 11, 8, 12, 0, 8, 8, 11, 7, 3, 9],  #4
    [27, 13, 12, 10, 8, 0, 2, 9, 3, 9, 8],  #5
    [28, 11, 12, 9, 9, 2, 0, 8, 3, 9, 6],   #6
    [28, 5, 8, 2, 10, 10, 8, 0, 6, 10, 3],  #7
    [28, 9, 10, 7, 7, 4, 3, 6, 0, 6, 4],    #8
    [23, 9, 6, 11, 3, 8, 8, 10, 6, 0, 7],   #9
    [25, 6, 6, 4, 8, 8, 7, 3, 5, 7, 0],     #10
]


def Manipulation(demand, distance):
    updated = {}
    Manipulated_Demand = []
    Manipulates_Distance = []
    for i in demand:
        if i % 2 == 0:
            Manipulated_Demand.extend([2] * math.floor(i / 2))
        else:
            Manipulated_Demand.extend([2] * (math.floor(i / 2) - 1))
            Manipulated_Demand.append(3)
    Manipulated_Demand.insert(0, 0)
    updated["Demand"] = Manipulated_Demand
    dict_inx = {}
    prev = 0
    for i, d in enumerate(demand):
        dict_inx.update({i + 1: (list(range(prev + 1, prev + 1 + math.floor(d / 2))))})
        prev += math.floor(d / 2)
    updated["Parent_MDC"] = dict_inx
    for i in range(1, len(Manipulated_Demand)):
        temp = [0]
        row = search_node(i, dict_inx)
        for j in range(1, len(Manipulated_Demand)):
            col = search_node(j, dict_inx)
            temp.append(distance[row][col])
        Manipulates_Distance.append(temp)

    temp = [0]
    for i in range(1, len(Manipulated_Demand)):
        col = search_node(i, dict_inx)
        temp.append(distance[0][col])
    Manipulates_Distance.insert(0, temp)
    updated["Distance"] = Manipulates_Distance
    updated["Parent_MDC"] = dict_inx
    return updated


def search_node(val, dict_inx):
    for node, node_list in dict_inx.items():
        if val in node_list:
            return node


'''
def Distance_Manipulation(distance, demand):

    Manipulates_Distance =[]
    print(demand)
    temp = [0]
    for i in range(len(demand)):
        temp = []
        for j,d in enumerate(demand):
            for k in range(math.floor(i / 2)):
                temp.append(distance[i+1][j+1])
        print(temp)
        for k in range(math.floor(i / 2)):
            Manipulates_Distance.append(temp)
    print(Manipulates_Distance)
    # return  Manipulates_Distance

fc_row = [0]
for i in range(len(demand)):
    temp = [0]
    for j in range(1,len(demand)):
        temp.append(distance[i][j])
        x = temp[-1]
        temp.extend([x] * (math.floor(demand[i] / 2) - 1))
    for k in range(math.floor(demand[i] / 2)):
        Manipulates_Distance.append(temp)
    fc_row.append(distance[0][i + 1])
    x = fc_row[-1]
    fc_row.extend([x] * (math.floor(demand[i] / 2) - 1))
Manipulates_Distance.insert(0, fc_row)
Manipulated_Demand.insert(0, 0)
print(Manipulated_Demand)
print(len(Manipulated_Demand))
for i in Manipulates_Distance:
    print(len(i),",,,,,",i)

def search_node(val):
    for node, node_list in dict_inx.items():
        if val in node_list:
            return node'''


def create_data_model(updated_data):
    data = {}
    data['distance_matrix'] = updated_data['Distance']
    data['num_vehicles'] = 15
    data['demands'] = updated_data["Demand"]
    data['vehicle_capacities'] = [28] * 15
    data['depot'] = 0
    data["cost_per_demand_unit"] = 0
    data["vehicle_VarCost"] = [10] * 15
    data['Fixed_cost'] = [1000] * 15
    return data


def print_solution(data, manager, routing, assignment, dic):
    total_distance = 0
    total_load = 0
    total_vehicle = 0
    t = []
    one = 0
    two = 0
    three = 0
    more_than_three = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        node = 0
        count_same = 1
        route = []
        while not routing.IsEnd(index):
            temp = []
            node_index = manager.IndexToNode(index)
            node += 1
            route_load += data['demands'][node_index]
            plan_output += ' {0} Load({1}) -> '.format(search_node(node_index, dic), route_load)
            previous_index = index
            index = assignment.Value(routing.NextVar(index))
            if search_node(previous_index, dic) == search_node(index, dic) and previous_index is not None:
                count_same += 1
            elif previous_index is not None:
                temp.append(search_node(previous_index, dic))
                temp.append(count_same)
                route.append(temp)
                count_same = 1
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        plan_output += ' {0} Load({1})\n'.format(search_node(manager.IndexToNode(index), dic),
                                                 route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        if len(route) == 2:
            one += 1
        elif len(route) == 3:
            two += 1
        elif len(route) == 4:
            three += 1
        elif len(route) > 4:
            more_than_three += 1
        if node > 1:
            print(plan_output)
            total_vehicle += 1
        total_distance += route_distance
        total_load += route_load
    result.append(total_distance)
    result.append(total_vehicle)
    result.append(one)
    result.append(two)
    result.append(three)
    result.append(more_than_three)
    print(result)


def main(updated_data):
    data = create_data_model(updated_data)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    routing = pywrapcp.RoutingModel(manager)

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

    evaluator_list = []

    def create_distance_callback1(data, v,f):
        # Create a callback to calculate distances between cities.

        def distance_callback(from_node, to_node):
            from_node = manager.IndexToNode(from_node)
            to_node = manager.IndexToNode(to_node)
            return int((data['distance_matrix'][from_node][to_node]) * v )

        # def fix_callback(data, veh_idx):
        #     print("inside", data['Fixed_cost'][veh_idx])
        #     return data['Fixed_cost'][veh_idx]
        #
        # print("test",distance_callback)
        # l.append(distance_callback)
        # l.append(fix_callback)

        return distance_callback



    for vr in range(data['num_vehicles']):
        print(data["vehicle_VarCost"][vr])
        dist_callback1 = create_distance_callback1(data, data["vehicle_VarCost"][vr],data['Fixed_cost'][vr])
        evaluator_list.append(dist_callback1)
        routing.SetFixedCostOfVehicle(data['Fixed_cost'][vr],vr)
        # evaluator_list.append(fix_callback1)
    for vehicle_id, evaluator in enumerate(evaluator_list):
        temp_cost = (routing.RegisterTransitCallback(evaluator))
        routing.SetArcCostEvaluatorOfVehicle(temp_cost, vehicle_id)

    # def distance_callback(from_node, to_node):
    #     from_node = manager.IndexToNode(from_node)
    #     to_node = manager.IndexToNode(to_node)
    #     print(from_node, "--->", to_node)
    #     if search_node(from_node) == search_node(to_node):
    #         print("sak", int(dist_matrix[from_node][to_node]) * 0)
    #     else:
    #         print("sak", int(dist_matrix[from_node][to_node]) * v)
    #     return int(dist_matrix[from_node][to_node]) * v
    #
    # def create_distance_callback1(dist_matrix, v):
    #     t = routing.RegisterTransitCallback(distance_callback)
    #     return t
    # l = []
    #
    # for vr in range(data['num_vehicles']):
    #     transit_callback_index = create_distance_callback1(data['distance_matrix'], data["vehicle_VarCost"][vr])
    #     l.append(transit_callback_index)
    #
    # for vehicle_id, val_cost in enumerate(l):
    #     transit_callback_index = routing.RegisterTransitCallback(create_distance_callback1)
    #     routing.SetArcCostEvaluatorOfVehicle(transit_callback_index, vehicle_id)

    plus_one_callback_index = routing.RegisterUnaryTransitCallback(lambda index: 1)
    count_dimension_name = 'count'
    routing.AddDimension(
        plus_one_callback_index,
        0,
        20,
        True,
        count_dimension_name
    )
    count_dimension = routing.GetDimensionOrDie(count_dimension_name)

    for vehicle_idx in range(0, data['num_vehicles']):
        index = routing.End(vehicle_idx)
        count_dimension.CumulVar(index).SetRange(0, 20)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        dic = updated_data['Parent_MDC']
        print_solution(data, manager, routing, assignment, dic)


if __name__ == '__main__':
    final_write = []
    for run in range(1):
        result = []
        demand = [random.randint(3, 10) for i in range(len(distance) - 1)]
        updated_data = Manipulation(demand, distance)
        print(run)
        total_slot = sum(demand)
        result.append(run)
        result.append(total_slot)
        result.append(math.ceil(total_slot / 28))
        main(updated_data)
        final_write.append(result)
    with open("out.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(final_write)
    # print(updated_data.keys())
    # main(updated_data)


        def travelcost_callback(from_node, to_node):
            from_node = manager.IndexToNode(from_node)
            to_node = manager.IndexToNode(to_node)
            return int((data['distance_matrix'][from_node][to_node]) * var)

        var_callback = travelcost_callback
        l.append(var_callback)

        def fixcost_callback(f):
            return f
        fix_calback = fixcost_callback
        l.append(fix_calback)
        for i, evaluator in enumerate(evaluator_list):
            temp_cost = (routing.RegisterTransitCallback(evaluator))

        return temp_cost

    for vr in range(data['num_vehicles']):
        cost_callback = create_cost_callback(data, data["vehicle_VarCost"][vr],data['Fixed_cost'][vr])
        evaluator_list.append(cost_callback)
    for vehicle_id, evaluator in enumerate(evaluator_list):
        temp_cost = (routing.RegisterTransitCallback(evaluator))
        print("temp",temp_cost)
        routing.SetArcCostEvaluatorOfVehicle(temp_cost, vehicle_id)
        # routing.SetArcCostEvaluatorOfVehicle(cost_callback, i)


    plus_one_callback_index = routing.RegisterUnaryTransitCallback(lambda index: 1)
    count_dimension_name = 'count'
    routing.AddDimension(
        plus_one_callback_index,
        0,
        20,
        True,
        count_dimension_name
    )
    count_dimension = routing.GetDimensionOrDie(count_dimension_name)

    for vehicle_idx in range(0, data['num_vehicles']):
        index = routing.End(vehicle_idx)
        count_dimension.CumulVar(index).SetRange(0, 20)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    assignment = routing.SolveWithParameters(search_parameters)

    if assignment:
        dic = updated_data['Parent_MDC']
        print_solution(data, manager, routing, assignment, dic)


if __name__ == '__main__':
    final_write = []
    for run in range(1):
        result = []
        demand = [random.randint(3, 10) for i in range(len(distance) - 1)]
        updated_data = Manipulation(demand, distance)
        print(run)
        total_slot = sum(demand)
        result.append(run)
        result.append(total_slot)
        result.append(math.ceil(total_slot / 28))
        main(updated_data)
        final_write.append(result)
    with open("out.csv", "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerows(final_write)
    # print(updated_data.keys())
    # main(updated_data)
