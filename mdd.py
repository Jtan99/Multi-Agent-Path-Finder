import heapq

class NodeDictWrap(dict):

    def __init__(self,*args, **kwargs):
        super().__init__(*args, **kwargs)

    def __getattr__(self, item):
        return super().__getitem__(item)

    def __setattr__(self, item, value):
        return super().__setitem__(item, value)

    def __lt__(self,other):
        # break ties in favor of lower h_val. 
        return self.h_val < other.h_val


def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], NodeDictWrap(node)))
    # print(f"PUSHING heuristic: {node['g_val'] + node['h_val']} path: {get_path(node)}")

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    # print(f"POPPING heuristic: {curr['g_val'] + curr['h_val']} path: {get_path(curr)}")
    return curr

def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_all_optimal_paths(my_map, start_loc, goal_loc, h_values):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
    """
    open_list = []
    closed_list = dict() #contains list of already expanded nodes
    earliest_goal_timestep = 0
    first_time = True
    paths = []
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'time': 0}
    push_node(open_list, root)
    closed_list[(root['loc'], root['time'])] = root
    while len(open_list) > 0:
        # print("open list to pop", open_list)
        curr = pop_node(open_list)
        # print("curr is:", get_path(curr))
        if curr['loc'] == goal_loc:
            curr_path = get_path(curr)
            print("Current path to goal found:", curr_path)
            if(first_time):
                earliest_goal_timestep = len(curr_path)
                first_time = False
            elif (len(curr_path) > earliest_goal_timestep):
                return paths
            paths.append(curr_path)
            continue

            
        for dir in range(4):
            child_loc = move(curr['loc'], dir)
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue            
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'time': curr['time'] + 1}
            
            closed_list[(child['loc'], child['time'])] = child
            push_node(open_list, child)

    return None  # Failed to find solutions
