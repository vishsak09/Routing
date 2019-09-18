import csv
import math

from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

# demand = [20, 20, 25, 20, 30, 20, 20, 20, 20, 20, ]
result = []

demand =  [13,8,3,3,6,6,13,3,8,6]
distance = [
[0, 12426, 8668, 9444, 8508, 16322, 11014, 7617, 15517, 9142, 12123],
[12690, 0, 7368, 4639, 14117, 6451, 2530, 12459, 3142, 8032, 3724],
[6716, 7017, 0, 5343, 7958, 12176, 5605, 6301, 9801, 5041, 3410],
[9654, 4790, 6518, 0, 10144, 10723, 7284, 8487, 4904, 4060, 1669],
[9775, 11349, 6049, 7006, 0, 17283, 10539, 2390, 11464, 6704, 7819],
[17494, 4169, 10230, 8756, 18234, 0, 5393, 16576, 7259, 12149, 7841],
[11244, 2524, 5922, 7112, 13390, 5971, 0, 11732, 5615, 10505, 4664],
[7743, 12599, 7298, 8255, 1249, 18532, 11788, 0, 12713, 7953, 9069],
[14017, 2940, 10881, 5030, 14507, 7738, 5434, 12850, 0, 8423, 5014],
[9126, 8132, 5990, 3789, 9616, 14066, 10627, 7959, 8247, 0, 4602],
[12381, 3729, 3755, 1669, 10714, 10144, 4692, 9057, 4836, 4629, 0],
]

# demand = [7, 3, 1, 4, 5, 2, 2, 4, 4]
# distance = [
#     [0, 12426, 8668, 11014, 8508, 9142, 7617, 13461, 16322, 12123],
#     [12690, 0, 7368, 2530, 14117, 8032, 12459, 1087, 6451, 3724],
#     [6716, 7017, 0, 5605, 7958, 5041, 6301, 8052, 12176, 3410],
#     [11244, 2524, 5922, 0, 13390, 10505, 11732, 3560, 5971, 4664],
#     [9775, 11349, 6049, 10539, 0, 6704, 2390, 10769, 17283, 7819],
#     [9126, 8132, 5990, 10627, 9616, 0, 7959, 7552, 14066, 4602],
#     [7743, 12599, 7298, 11788, 1249, 7953, 0, 12018, 18532, 9069],
#     [13723, 1069, 8401, 3563, 13541, 7456, 11883, 0, 7484, 3148],
#     [17494, 17494, 4169, 10230, 5393, 18234, 12149, 8756, 0, 35233],
#     [12381, 12381, 3729, 3755, 4692, 10714, 4629, 1669, 34659, 0],
# ]


def print_solution(data, manager, routing, assignment, dic):
    total_distance = 0
    total_load = 0
    total_vehicle = 0
    t = []
    one = 0
    two = 0
    three = 0
    more_than_three = 0
    keys = list(set(data['vehicle_capacities']))
    vehType_dict = {key: 0 for key in keys}
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        distance_covered = 0
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
        plan_output += 'Cost of the route: {}Rs.\n'.format(route_distance)

        print(plan_output)
        if len(route) == 2:
            one += 1
        elif len(route) == 3:
            two += 1
        elif len(route) == 4:
            three += 1
        elif len(route) > 4:
            more_than_three += 1
        if node > 1:
            vehType_dict[data['vehicle_capacities'][vehicle_id]] += 1
            total_vehicle += 1
        total_distance += route_distance
        total_load += route_load
    result.append(total_distance)
    result.append(total_vehicle)
    result.append(one)
    result.append(two)
    result.append(three)
    result.append(more_than_three)
    result.append(vehType_dict)
    print("Total cost: ", total_distance)
    print("Total vehicle: ", total_vehicle)
    # print(vehType_dict)
    # print(result)


def Manipulation(demand, distance):
    updated = {}
    Manipulated_Demand = []
    Manipulates_Distance = []
    for i in demand:
        if i == 1:
            Manipulated_Demand.extend([1])
        elif i % 2 == 0:
            Manipulated_Demand.extend([2] * int(math.floor(i / 2)))
        else:
            Manipulated_Demand.extend([2] * int(math.floor(i / 2) - 1))
            Manipulated_Demand.append(3)
    Manipulated_Demand.insert(0, 0)
    updated["Demand"] = Manipulated_Demand
    print(updated["Demand"])
    dict_inx = {}
    prev = 0
    for i, d in enumerate(demand):
        if d == 1:
            t = prev + 1
            dict_inx.update({i + 1: [t]})
            prev += 1
        else:
            dict_inx.update({i + 1: (list(range(prev + 1, int(prev + 1 + math.floor(d / 2)))))})
            prev += int(math.floor(d / 2))
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
    # print (updated["Distance"])
    # updated["Parent_MDC"] = dict_inx
    return updated


def search_node(val, dict_inx):
    if val == 0:
        return 0
    for node, node_list in dict_inx.items():
        if val in node_list:
            return node

def create_data_model(updated_data):
    data = {}
    data['distance_matrix'] = updated_data['Distance']
    data['num_vehicles'] = 10
    data['demands'] = updated_data["Demand"]
    data['vehicle_capacities'] = [28] * 5
    data['vehicle_capacities'].extend([11] * 5)
    data['depot'] = 0
    data["cost_per_demand_unit"] = 0
    data["vehicle_VarCost"] = [13] * 5
    data['vehicle_VarCost'].extend([10] * 5)
    data['Fixed_cost'] = [2500] * 5
    data['Fixed_cost'].extend([1800] * 5)

    return data



def main(updated_data):
    data = create_data_model(updated_data)
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    routing = pywrapcp.RoutingModel(manager)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')

    evaluator_list = []

    def create_distance_callback(data, v):
        # Create a callback to calculate distances between cities.

        def distance_callback(from_node, to_node):
            from_node = manager.IndexToNode(from_node)
            to_node = manager.IndexToNode(to_node)
            #print("Finding dist b/w ", from_node, " and ", to_node)
            return int((data['distance_matrix'][from_node][to_node]) * v * 2 / 1000)

        return distance_callback

    for vehicle_id in range(data['num_vehicles']):
        # print(data["vehicle_VarCost"][vr])
        dist_callback = create_distance_callback(data, data["vehicle_VarCost"][vehicle_id])
        dist_callback_idx = routing.RegisterTransitCallback(dist_callback)
        routing.SetFixedCostOfVehicle(data['Fixed_cost'][vehicle_id], vehicle_id)
        routing.SetArcCostEvaluatorOfVehicle(dist_callback_idx, vehicle_id)

    def city_count_restriction(updated_data):
        def underlying(from_node, to_node):
            from_node = manager.IndexToNode(from_node)
            to_node = manager.IndexToNode(to_node)
            #print(" from ", from_node, " to ", to_node)
            if to_node == 0:
                return 0
            if search_node(from_node, updated_data['Parent_MDC']) != search_node(to_node, updated_data['Parent_MDC']):
                #print(from_node, to_node, " are apart")
                return 1
            else:
                #print(from_node, to_node, " are together")
                return 0
        return underlying

    arc = routing.RegisterTransitCallback(city_count_restriction(updated_data))
    routing.AddDimension(
            arc,
            0,
            3, # Max cities visited
            True,
            'visited_count_vehicle_'
         )

    def city_loop_restriction(updated_data, cid):
        def underlying(from_node, to_node):
            from_node = manager.IndexToNode(from_node)
            to_node = manager.IndexToNode(to_node)

            #print("city ", cid, " from ", from_node, " to ", to_node)
            if search_node(to_node, updated_data['Parent_MDC']) == cid and search_node(from_node, updated_data['Parent_MDC']) != search_node(to_node, updated_data['Parent_MDC']):
                return 1
            else:
                return 0
        return underlying

    for cid in updated_data['Parent_MDC'].keys():
        print("Adding loop constraint for city ", cid)
        arc = routing.RegisterTransitCallback(city_loop_restriction(updated_data, cid))
        routing.AddDimension(
            arc,
            0,
            1,
            True,
            'loop_count_city_' + str(cid) + '_vehicle_'
         )

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.time_limit.seconds=6 # You can increase this time
    search_parameters.log_search = True # Will disable or-tools logging
    #search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION) # not much useful
    search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)

    assignment = routing.SolveWithParameters(search_parameters)

    print("Solver status: ", routing.status())

    if assignment:
        dic = updated_data['Parent_MDC']
        print_solution(data, manager, routing, assignment, dic)


if __name__ == '__main__':
    final_write = []
    for run in range(1):
        result = []
        updated_data = Manipulation(demand, distance)
        print(run)
        total_slot = sum(demand)
        result.append(run)
        result.append(total_slot)
        result.append(math.ceil(total_slot / 28))
        main(updated_data)
        final_write.append(result)
    with open("Mumbai_13-Sept.csv", "w") as f:
        writer = csv.writer(f)
        writer.writerows(final_write)
    # print(updated_data.keys())
    # main(updated_data)