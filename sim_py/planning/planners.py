import numpy as np
import matplotlib.pyplot as plt
from misc import Timer, latlong_distance
from queues import FIFO, LIFO, PriorityQueue

def depth_first(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    """Depth first planner."""
    t = Timer()
    t.tic()
    
    unvis_node = -1
    previous = np.full(num_nodes, dtype=int, fill_value=unvis_node)
    cost_to_come = np.zeros(num_nodes)
    control_to_come = np.zeros((num_nodes, num_controls), dtype=int)
    expanded_nodes = []
    
    startNode = mission['start']['id']
    goalNode = mission['goal']['id']

    q = LIFO()
    q.insert(startNode)
    foundPlan = False

    while not q.IsEmpty():
        x = q.pop()
        expanded_nodes.append(x)
        if x == goalNode:
            foundPlan = True
            break
        neighbours, u, d = f_next(x)
        for xi, ui, di in zip(neighbours, u, d):
            if previous[xi] == unvis_node:
                previous[xi] = x
                q.insert(xi)
                cost_to_come[xi] = cost_to_come[x] + di
                if num_controls > 0:
                    control_to_come[xi] = ui

    # Recreate the plan by traversing previous from goal node
    if not foundPlan:
        return []
    else:
        plan = [goalNode]
        length = cost_to_come[goalNode]
        control = []
        while plan[0] != startNode:
            if num_controls > 0:
                control.insert(0, control_to_come[plan[0]])
            plan.insert(0, previous[plan[0]])

        return {'plan': plan,
                'length': length,
                'num_expanded_nodes': len(expanded_nodes),
                'name': 'DepthFirst',
                'time': t.toc(),
                'control': control,
                'expanded_nodes': expanded_nodes}

def breadth_first(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    """Depth first planner."""
    t = Timer()
    t.tic()
    
    unvis_node = -1
    previous = np.full(num_nodes, dtype=int, fill_value=unvis_node)
    cost_to_come = np.zeros(num_nodes)
    control_to_come = np.zeros((num_nodes, num_controls), dtype=int)
    expanded_nodes = []
    
    startNode = mission['start']['id']
    goalNode = mission['goal']['id']

    q = FIFO()
    q.insert(startNode)
    foundPlan = False

    while not q.IsEmpty():
        x = q.pop()
        expanded_nodes.append(x)
        if x == goalNode:
            foundPlan = True
            break
        neighbours, u, d = f_next(x)
        for xi, ui, di in zip(neighbours, u, d):
            if previous[xi] == unvis_node:
                previous[xi] = x
                q.insert(xi)
                cost_to_come[xi] = cost_to_come[x] + di
                if num_controls > 0:
                    control_to_come[xi] = ui

    # Recreate the plan by traversing previous from goal node
    if not foundPlan:
        return []
    else:
        plan = [goalNode]
        length = cost_to_come[goalNode]
        control = []
        while plan[0] != startNode:
            if num_controls > 0:
                control.insert(0, control_to_come[plan[0]])
            plan.insert(0, previous[plan[0]])

        return {'plan': plan,
                'length': length,
                'num_expanded_nodes': len(expanded_nodes),
                'name': 'DepthFirst',
                'time': t.toc(),
                'control': control,
                'expanded_nodes': expanded_nodes}

def dijkstra(num_nodes, mission, f_next, heuristic=None, num_controls=0):
    """Dijkstra planner."""
    t = Timer()
    t.tic()
    
    unvis_node = -1
    previous = np.full(num_nodes, dtype=int, fill_value=unvis_node)
    cost_to_come = np.zeros(num_nodes)
    control_to_come = np.zeros((num_nodes, num_controls), dtype=int)
    expanded_nodes = []
    
    startNode = mission['start']['id']
    goalNode = mission['goal']['id']

    q = PriorityQueue()
    q.insert(startNode, 0)
    foundPlan = False

    while not q.IsEmpty():
        value, prio = q.pop()
        expanded_nodes.append(value)
        if value == goalNode:
            foundPlan = True
            break
        neighbours, u, d = f_next(value)
        for xi, ui, di in zip(neighbours, u, d):
            if previous[xi] == unvis_node or cost_to_come[xi] > cost_to_come[value] + di:
                previous[xi] = value
                cost_to_come[xi] = cost_to_come[value] + di
                q.insert(xi, cost_to_come[xi] )
                if num_controls > 0:
                    control_to_come[xi] = ui

    # Recreate the plan by traversing previous from goal node
    if not foundPlan:
        return []
    else:
        plan = [goalNode]
        length = cost_to_come[goalNode]
        control = []
        while plan[0] != startNode:
            if num_controls > 0:
                control.insert(0, control_to_come[plan[0]])
            plan.insert(0, previous[plan[0]])

        return {'plan': plan,
                'length': length,
                'num_expanded_nodes': len(expanded_nodes),
                'name': 'DepthFirst',
                'time': t.toc(),
                'control': control,
                'expanded_nodes': expanded_nodes}

def astar(num_nodes, mission, f_next, heuristic=latlong_distance, num_controls=0):
    """astra planner."""
    t = Timer()
    t.tic()
    
    unvis_node = -1
    previous = np.full(num_nodes, dtype=int, fill_value=unvis_node)
    cost_to_come = np.zeros(num_nodes)
    control_to_come = np.zeros((num_nodes, num_controls), dtype=int)
    expanded_nodes = []
    
    startNode = mission['start']['id']
    goalNode = mission['goal']['id']

    q = PriorityQueue()
    q.insert(startNode, heuristic(mission['start']['id'], mission['goal']['id']) )
    foundPlan = False

    while not q.IsEmpty():
        x, prio = q.pop()
        expanded_nodes.append(x)
        if x == goalNode:
            foundPlan = True
            break
        neighbours, u, d = f_next(x)
        for xi, ui, di in zip(neighbours, u, d):
            if previous[xi] == unvis_node or cost_to_come[xi] > cost_to_come[x] + di:
                previous[xi] = x
                cost_to_come[xi] = cost_to_come[x] + di
               
                q.insert(xi,  heuristic(xi , mission['goal']['id']) + cost_to_come[xi] )
                if num_controls > 0:
                    control_to_come[xi] = ui

    # Recreate the plan by traversing previous from goal node
    if not foundPlan:
        return []
    else:
        plan = [goalNode]
        length = cost_to_come[goalNode]
        control = []
        while plan[0] != startNode:
            if num_controls > 0:
                control.insert(0, control_to_come[plan[0]])
            plan.insert(0, previous[plan[0]])

        return {'plan': plan,
                'length': length,
                'num_expanded_nodes': len(expanded_nodes),
                'name': 'DepthFirst',
                'time': t.toc(),
                'control': control,
                'expanded_nodes': expanded_nodes}

def best_first(num_nodes, mission, f_next, heuristic, num_controls=0):
    """best first planner."""
    t = Timer()
    t.tic()
    
    unvis_node = -1
    previous = np.full(num_nodes, dtype=int, fill_value=unvis_node)
    cost_to_come = np.zeros(num_nodes)
    control_to_come = np.zeros((num_nodes, num_controls), dtype=int)
    expanded_nodes = []
    
    startNode = mission['start']['id']
    goalNode = mission['goal']['id']

    q = PriorityQueue()
    q.insert(startNode, heuristic(mission['start']['id'], mission['goal']['id']) )
    foundPlan = False

    while not q.IsEmpty():
        x, prio = q.pop()
        expanded_nodes.append(x)
        if x == goalNode:
            foundPlan = True
            break
        neighbours, u, d = f_next(x)
        for xi, ui, di in zip(neighbours, u, d):
            if previous[xi] == unvis_node:
                previous[xi] = x
                cost_to_come[xi] = cost_to_come[x] + di
               
               
                q.insert(xi,  heuristic(xi, mission['goal']['id']))
                if num_controls > 0:
                    control_to_come[xi] = ui

    # Recreate the plan by traversing previous from goal node
    if not foundPlan:
        return []
    else:
        plan = [goalNode]
        length = cost_to_come[goalNode]
        control = []
        while plan[0] != startNode:
            if num_controls > 0:
                control.insert(0, control_to_come[plan[0]])
            plan.insert(0, previous[plan[0]])

        return {'plan': plan,
                'length': length,
                'num_expanded_nodes': len(expanded_nodes),
                'name': 'DepthFirst',
                'time': t.toc(),
                'control': control,
                'expanded_nodes': expanded_nodes}
# ## Planning example using the DepthFirst planner

