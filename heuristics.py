from mdd import *
from mvc import *
from single_agent_planner import a_star, get_sum_of_cost
from cbs_cg import CBSSolver

def get_cg_heuristic(my_map, paths, starts, goals, low_level_h, constraints):
    cardinal_conflicts = []

    # build mdd for every agent
    # store reuse it later
    all_paths = []
    all_mdds = []
    for i in range(len(paths)):
        paths = get_all_optimal_paths(my_map, starts[i], goals[i], low_level_h[i], i, constraints)
        _, nodes_dict = buildMDDTree(paths)
        all_paths.append(paths)
        all_mdds.append(nodes_dict)
        


    for i in range(len(paths)): # num of agents in map
        # paths1 = get_all_optimal_paths(my_map, starts[i], goals[i], low_level_h[i])
        # print(f"All optimal paths for agent {i}: {paths1}")
        # root1, nodes_dict1 = buildMDDTree(paths1)
        # displayLayer(root1, 0)
        paths1 = all_paths[i]
        nodes_dict1 = all_mdds[i]

        for j in range(i+1,len(paths)):
            # paths2 = get_all_optimal_paths(my_map, starts[j], goals[j], low_level_h[j])
            # print(f"All optimal paths for agent {j}: {paths2}")
            # root2, nodes_dict2 = buildMDDTree(paths2)
            paths2 = all_paths[j]
            nodes_dict2 = all_mdds[j]

            balanceMDDs(paths1, paths2, nodes_dict1, nodes_dict2)
            
            if (check_MDDs_for_conflict(nodes_dict1, nodes_dict2)):
                cardinal_conflicts.append((i,j))
    # print("Cardinal conflicts found:", cardinal_conflicts)
    
    g = Graph(len(paths))
    for conflict in cardinal_conflicts:
        g.addEdge(conflict[0], conflict[1])
    vertex_cover = g.getVertexCover()
    return len(vertex_cover)

def check_MDDs_for_conflict(node_dict1, node_dict2):
    # given two dicts
    # indexed like dict[location] = node in mdd tree
    # build two dicts
    # indexed like dict[timestep] = [ locations ]

    dict1 = {} # timestep: [location]
    dict2 = {}
    for (loc,time) in node_dict1.keys():
        # build dict
        if time in dict1: # if this timestep was original mdd
            dict1[time].append(loc)
        else: 
            dict1[time] = [loc]

    for (loc,time) in node_dict2.keys():
        # build dict
        if time in dict2:
            dict2[time].append(loc)
        else:
            dict2[time] = [loc]

    # lst = list(dict1.keys())
    # missing_timesteps_1 = [x for x in range(lst[0], lst[-1]+1) if x not in lst] # returns the missing timesteps
    # lst = list(dict2.keys())
    # missing_timesteps_2 = [x for x in range(lst[0], lst[-1]+1) if x not in lst] # returns the missing timesteps

    # for timestep in missing_timesteps_1:
    #     dict1[timestep] = dict1[timestep-1]
    # for timestep in missing_timesteps_2:
    #     dict2[timestep] = dict2[timestep-1]

    #extend dictionary
    diff = abs(len(dict1) - len(dict2))
    if len(dict1) == len(dict2):
        pass
    elif len(dict1) > len(dict2): # mdd2/dict2 is shorter
        # extend dict2 by diff
        last_timestep = max(dict2.keys())   # last timestep = 47
        for i in range(diff):               # i = 0 1 2 3
            dict2[last_timestep+i+1] = dict2[last_timestep]  # new timesteps = 48 49 50 51
    else:
        #extend dict1
        last_timestep = max(dict1.keys())
        for i in range(diff):
            dict1[last_timestep+i+1] = dict1[last_timestep]

    for time, locs in dict1.items():
        if (len(locs) == 1 and locs == dict2[time]):
            return True #found a cardinal

    return False

def balanceMDDs(paths1, paths2, node_dict1, node_dict2):
    height1 = len(paths1[0])
    height2 = len(paths2[0])

    if (height1 != height2):
        if (height1 < height2):
            # first mdd shorter
            goal_loc = paths1[0][-1]
            bottom_node = node_dict1[(goal_loc, height1-1)]
            extendMDDTree(bottom_node, height2-height1)
            
        else:
            # second is shorter
            goal_loc = paths2[0][-1]
            bottom_node = node_dict2[(goal_loc, height2-1)]
            extendMDDTree(bottom_node, height1-height2)

def get_dg_heuristic(my_map, paths, starts, goals, low_level_h, constraints):
    dependencies = []

    all_roots = []
    all_paths = []
    all_mdds = []
    for i in range(len(paths)):
        path = get_all_optimal_paths(my_map, starts[i], goals[i], low_level_h[i], i, constraints)
        root, nodes_dict = buildMDDTree(path)
        all_roots.append(root)
        all_paths.append(path)
        all_mdds.append(nodes_dict)

    for i in range(len(paths)): # num of agents in map
        # paths1 = get_all_optimal_paths(my_map, starts[i], goals[i], low_level_h[i], i, constraints)
        # print(f"All optimal paths for agent {i}: {paths1}")
        paths1 = all_paths[i]
        root1 = all_roots[i]
        node_dict1 = all_mdds[i]

        for j in range(i+1,len(paths)):
            # paths2 = get_all_optimal_paths(my_map, starts[j], goals[j], low_level_h[j], j, constraints)
            # print(f"All optimal paths for agent {j}: {paths2}")
            paths2 = all_paths[j]
            root2 = all_roots[j]
            node_dict2 = all_mdds[j]
            
            root, bottom_node = buildJointMDD(paths1, paths2, root1, node_dict1, root2, node_dict2)

            if (check_jointMDD_for_dependency(bottom_node, paths1, paths2)):
                dependencies.append((i,j))

    g = Graph(len(paths))
    for conflict in dependencies:
        g.addEdge(conflict[0], conflict[1])
    vertex_cover = g.getVertexCover()
    return len(vertex_cover)

def check_jointMDD_for_dependency(bottom_node, paths1, paths2):
    optimal_time = max(len(paths1),len(paths2))
    goal_loc1 = paths1[-1]
    goal_loc2 = paths2[-1]
    if (bottom_node.location != [(goal_loc1),(goal_loc2)] or bottom_node.timestep != optimal_time):
        return True
    return False

def get_wdg_heuristic(my_map, paths, starts, goals, low_level_h, constraints):
    dependencies = []

    all_roots = []
    all_paths = []
    all_mdds = []

    for i in range(len(paths)):
        path = get_all_optimal_paths(my_map, starts[i], goals[i], low_level_h[i], i, constraints)
        root, nodes_dict = buildMDDTree(path)
        all_roots.append(root)
        all_paths.append(path)
        all_mdds.append(nodes_dict)

    for i in range(len(paths)): # num of agents in map
        paths1 = all_paths[i]
        root1 = all_roots[i]
        node_dict1 = all_mdds[i]

        for j in range(i+1,len(paths)):
            paths2 = all_paths[j]
            root2 = all_roots[j]
            node_dict2 = all_mdds[j]
            
            root, bottom_node = buildJointMDD(paths1, paths2, root1, node_dict1, root2, node_dict2)

            if (check_jointMDD_for_dependency(bottom_node, paths1, paths2)):
                dependencies.append((i,j))

    g = WeightedGraph(list(range(len(paths))))
    for dependency in dependencies:
        agent1 = dependency[0]
        agent2 = dependency[1]
        new_starts = [starts[agent1], starts[agent2]]
        new_goals = [goals[agent1], goals[agent2]]
        cbs = CBSSolver(my_map, new_starts, new_goals)
        paths = cbs.find_solution_cg(root_constraints=constraints, root_h=1)
        min_cost = get_sum_of_cost(paths)
        sum_indv_opt_paths = len(all_paths[agent1][0]) + len(all_paths[agent2][0])
        edge_weight = sum_indv_opt_paths - min_cost

        g.add_edge(dependency[0], dependency[1], edge_weight)

    # g is the wdg graph
    # either create our own branch and bound
    # or figure out how to get an ilp solver working

    #       [i] ---- 4 ---- [j]
    #         \             /
    #           3          2
    #             \ [k]  /
#       2 variables/vertex i, j 

   #       [i] ---- 4 ---- [j]
    #         \             /
    #           1          1
    #             \ [k]  /

# node in the bnb queue                                                     (infinity, infinity)
# iter 1- branch on all possible values of 1 variable (i)           (0, infinity, infinity) (1, infinity, infinity) (2, infinity, infinity) (3, infinity) (4, infinity) 
                    #                                                   (0,4, infinity)       (1,3,infinity)       (2,2)           (3,1)       (4,0)
                                                                # (0, 4, 3) 

# a branch-and-bound algorithm that branches on the possible
# values of each xi
# in the component and prunes nodes using
# the cost of the best result so far.
    return 0
