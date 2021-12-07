from mdd import *
from mvc import *

def get_cg_heuristic(my_map, paths, starts, goals, low_level_h):
    cardinal_conflicts = []

    for i in range(len(paths)): # num of agents in map
        paths1 = get_all_optimal_paths(my_map, starts[i], goals[i], low_level_h[i])
        # print(f"All optimal paths for agent {i}: {paths1}")
        root1, nodes_dict1 = buildMDDTree(paths1)
        # displayLayer(root1, 0)

        for j in range(i+1,len(paths)):
            paths2 = get_all_optimal_paths(my_map, starts[j], goals[j], low_level_h[j])
            # print(f"All optimal paths for agent {j}: {paths2}")
            root2, nodes_dict2 = buildMDDTree(paths2)
            
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
    dict1 = {}
    dict2 = {}

    for loc, node in node_dict1.items():
        # build dict
        if node.timestep in dict1:
            dict1[node.timestep].append(loc)
        else:
            dict1[node.timestep] = [loc]
    for loc, node in node_dict2.items():
        # build dict
        if node.timestep in dict2:
            dict2[node.timestep].append(loc)
        else:
            dict2[node.timestep] = [loc]

    #extend dictionary
    diff = abs(len(dict1) - len(dict2))
    if len(dict1) == len(dict2):
        pass
    elif len(dict1) > len(dict2):
        # extend dict2 by diff
        last_timestep = max(dict2.keys())
        for i in range(diff):
            dict2[last_timestep+i+1] = dict2[last_timestep]
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
            bottom_node = node_dict1[goal_loc]
            extendMDDTree(bottom_node, height2-height1)
            
        else:
            # second is shorter
            goal_loc = paths2[0][-1]
            bottom_node = node_dict2[goal_loc]
            extendMDDTree(bottom_node, height1-height2)
