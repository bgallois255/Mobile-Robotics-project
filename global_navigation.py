import matplotlib.pyplot as plt
from matplotlib.legend_handler import HandlerTuple
import numpy as np
import time
from shapely.geometry import Point, Polygon, LineString, MultiPolygon

def dijkstra(graph, start, goal):
    """
    @parameters: -graph: a dictionnary having entries for each vertex of the environment connected to a list of tuples containing the coordinates of each vertex it can connect to, along with the length of the path to that vertex 
                 -start: tuple, robot position coordinates
                 -goal: tuple, goal position coordinates

    @description: this function implements the dijkstra algorithm to find the shortest path from the robot position to the goal position, only travelling from vertex to vertex

    @exit: list of coordinates tuples indicating the successive vertices to pass by in order to go from initial to goal position by travelling the shortest distance

    """


    start = tuple(map(float, start))
    goal = tuple(map(float, goal))

    shortest_paths = {start: (None, 0)}
    current_node = start
    visited = set()

    while current_node != goal:
        visited.add(current_node)
        destinations = graph.get(current_node, [])
        weight_to_current_node = shortest_paths[current_node][1]

        for next_node, weight in destinations:
            weight = weight_to_current_node + weight
            next_node_key = tuple(map(float, next_node))
            if next_node_key not in shortest_paths or weight < shortest_paths[next_node_key][1]:
                shortest_paths[next_node_key] = (current_node, weight)

        next_destinations = {node: shortest_paths[node][1] for node in shortest_paths if node not in visited}
        if not next_destinations:
            print("Path not reachable")
            return "Path not reachable"

        current_node = min(next_destinations, key=next_destinations.get)

    path = []
    while current_node is not None:
        path.append(current_node)
        next_node = shortest_paths[current_node][0]
        current_node = next_node

    return path[::-1]



def visibility_graph(p_robot, obstacles, p_goal):
    """
    @parameters: -p_robot: tuple, robot position coordinates
                 -obstacles: a list of lists of organized coordinate tuples indicating the succesive corners (clockwise or anticlockwise order) of the safe zones corresponding to each obstacles detected by the vision algorithm
                 -p_goal: tuple, goal position coordinates
                 

    @description: the function creates the graph of the virtual environment by first generating a list of all vertices (robot, goal or obstacle corner) in the environment.
                  Then, it creates a list of all edges not intersecting with any obstacle, meaning it only contains the fully visible ones. Finally it builds the graph.

    @exit: -graph: a dictionnary having entries for each vertex of the environment connected to a list of tuples containing the coordinates of each vertex it can connect to, along with the length of the path to that vertex
           -edges: list of tuples of tuples: each tuple is a visible edge, containing two coordinate tuples indicating its start and finsish coordinates
    """
    all_vertices = [p_robot, p_goal]

    for obstacle in obstacles:
        all_vertices.extend(obstacle)

    # Remove duplicates while preserving order
    all_vertices = list(dict.fromkeys(all_vertices))

    edges = []


    # Fills the edges list with all edges that do not intersect with any obstacle
    for i, vertex1 in enumerate(all_vertices):
        for j, vertex2 in enumerate(all_vertices):
            if i != j:
                line = LineString([vertex1, vertex2])

                intersects_obstacle = any(line.crosses(Polygon(obstacle)) or line.within(Polygon(obstacle)) for obstacle in obstacles if not line.touches(Polygon(obstacle)))


                if not intersects_obstacle:
                    edges.append((vertex1, vertex2))

    edges = list(dict.fromkeys(edges)) #removes any eventual duplicate

    # Convert edges to a dictionary representation of the graph
    graph = {}
    for edge in edges:
        edge = (tuple(map(float, edge[0])), tuple(map(float, edge[1])))
        if edge[0] not in graph:
            graph[edge[0]] = []
        graph[edge[0]].append((edge[1], np.linalg.norm(np.array(edge[0]) - np.array(edge[1]))))
        if edge[1] not in graph:
            graph[edge[1]] = []
        graph[edge[1]].append((edge[0], np.linalg.norm(np.array(edge[0]) - np.array(edge[1]))))

    return graph, edges



def merge_overlapping_obstacles(obstacles):
    """
    @parameters: -obstacles: a list of lists of organized coordinate tuples indicating the succesive corners (clockwise or anticlockwise order) of the safe zones corresponding to each obstacles detected by the vision algorithm
                 

    @description: the function merges all overlapping obstacles into a single obstacle in order to reduce the amount of vertices given to the following algorithms and especially remove the redundant and useless ones

    @exit: An updated list of obstacles with possibly a shortest length, ensuring all obstacles are distinct from each other (but they can touch as long as they don't overlap)
    """
    polygons = [Polygon(obstacle) for obstacle in obstacles]
    for i in range(len(polygons)):
        if i > len(polygons) - 1 : break
        for polygon2 in polygons:
            if polygons[i].overlaps(polygon2):
                polygons[i] = polygons[i].union(polygon2)
                polygons.remove(polygon2)
    return [list(polygon.exterior.coords) for polygon in polygons]


def visualize_workspace(vertices, obstacles, p_robot, p_goal, edges, graph, shortest_path=None):
    """
    @description: this function simply plots the results in the virtual environment

    """

    plt.figure(figsize=(10, 10))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(-1, 20)
    plt.ylim(-1, 15)

    for i, obstacle in enumerate(obstacles):
        # Extract the exterior coordinates of the obstacle
        obstacle_array = np.array(obstacle.exterior.xy).T
        if i==0:
            plt.fill(obstacle_array[:, 0], obstacle_array[:, 1], color='gray', alpha=0.7, label = 'Obstacles')
        else: plt.fill(obstacle_array[:, 0], obstacle_array[:, 1], color='gray', alpha=0.7)

    

    for i, vertex in enumerate(vertices):
        if i==0:
            plt.plot(vertex[0], vertex[1], 'bo', label = 'Vertices')
        else: plt.plot(vertex[0], vertex[1], 'bo')

    plt.plot(p_robot[0], p_robot[1], 'ro', markersize=10, label='Robot')
    plt.plot(p_goal[0], p_goal[1], 'go', markersize=10, label='Goal')

    for i, edge in enumerate(graph.keys()):
        if i==0:
            for j, (neighbor, _) in enumerate(graph[edge]):
                if j==0:
                    plt.plot([edge[0], neighbor[0]], [edge[1], neighbor[1]], 'b-', label = 'Allowed Edges')
                else: plt.plot([edge[0], neighbor[0]], [edge[1], neighbor[1]], 'b-')

        else: 
            for neighbor, _ in graph[edge]:
                plt.plot([edge[0], neighbor[0]], [edge[1], neighbor[1]], 'b-')
 

    if shortest_path and shortest_path != "Path not reachable":
        # Plot the shortest path segment by segment
        for i in range(len(shortest_path) - 1):
            node1 = shortest_path[i]
            node2 = shortest_path[i + 1]
            if i==0:
                plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'r-', linewidth=2, label = 'Shortest Path')               
            else: plt.plot([node1[0], node2[0]], [node1[1], node2[1]], 'r-', linewidth=2)

    #plt.legend(['Obstacles', 'Robot', 'Goal', 'All Edges', 'Shortest Path'])
    plt.legend()
    plt.show()

def global_navigation(p_robot, obstacles, p_goal, merge_need, visualize):
    """
    @parameters: -p_robot: tuple, robot position coordinates
                 -obstacles: a list of lists of organized coordinate tuples indicating the succesive corners (clockwise or anticlockwise order) of the safe zones corresponding to each obstacles detected by the vision algorithm
                 -p_goal: tuple, goal position coordinates
                 -merge_need: Boolean, true if we want to recheck the overlapping of obstacles (if we have added an obstacle for instance)
                 -visualize: Boolean, true if visualization is needed
                 

    @description: The function implements the dijkstra, visibility_graph, merge_overlapping and visualize_workspace functions to solve the global navigation problem and potentially visualize the solution

    @exit: -shortest_path: list of coordinates tuples indicating the successive vertices to pass by in order to go from initial to goal position by travelling the shortest distance
           -obstacles: An updated list of obstacles with possibly a shortest length, ensuring all obstacles are distinct from each other (but they can touch as long as they don't overlap)
    """
    if(merge_need):
        obstacles = merge_overlapping_obstacles(obstacles)
    graph, edges = visibility_graph(p_robot, obstacles, p_goal)
    shortest_path = dijkstra(graph, p_robot, p_goal)
    if(visualize):
        visualize_workspace([p_robot, p_goal] + sum(obstacles, []), [Polygon(obs) for obs in obstacles],
                    p_robot, p_goal, edges, graph, shortest_path)
    return shortest_path, obstacles




"""
# Example usage:
p_robot = (0, 2)
obstacles = [
    [(2, 2), (3, 2), (3, 3.5), (4, 3), (4,4), (2,4)],
    [(6, 3), (6, 5), (8, 5), (8, 3)],
    [(4,0), (5,0), (5,2), (4,2)]
]
p_goal = (10, 5)

p_robot = (0,8)
obstacles = [
    [(1,1),(5,1),(5,2),(4,3),(4,7),(3,7),(3,6),(2,6), (2,8),(8,8),(8,9),(1,9)],
    [(6,1), (11,1), (11,9),(9,9),(10,8),(10,4),(9,4),(9,5),(8,5),(8,3),(7,3),(7,5),(6,5),(6,6),(9,6),(9,7),(5,7),(5,4),(6,3)]
]
obstacles = [[(0,0), (0,1), (1,1), (1,0)], [(1,0),(1,1),(3,1),(3,0)],  [(2,2), (2,3),(3,3), (3,2)]]
p_goal = (7.5,3.5)


#stage 1

p_robot = (0,4)
obstacles = [
    [(2,3.5),(3,3.5),(3,7),(5,7),(5,6),(6,6),(6,8),(2,8)],
    [(5,6),(6,6),(6,4),(5,4)],
    [(5,4),(6,4),(6,1),(4,1),(4,2),(5,2)],
    [(6,1),(4,1),(4,0),(6,0)]
]
p_goal = (8,4)

#Stage 2

p_robot = (4.5,4)
obstacles = [
    [(2,3.5),(3,3.5),(3,7),(5,7),(5,6),(6,6),(6,8),(2,8)],
    [(5,6),(6,6),(6,4),(5,4)],
    [(5,4),(6,4),(6,1),(4,1),(4,2),(5,2)],
    [(6,1),(4,1),(4,0),(6,0)]
]
obstacles.append([(4.5,3.5),(4.5,4.5),(6.5,4.5),(6.5,3.5)])
p_goal = (8,4)

#Stage 3

p_robot = (4.75,5.5)
obstacles = [
    [(2,3.5),(3,3.5),(3,7),(5,7),(5,6),(6,6),(6,8),(2,8)],
    [(5,6),(6,6),(6,4),(5,4)],
    [(5,4),(6,4),(6,1),(4,1),(4,2),(5,2)],
    [(6,1),(4,1),(4,0),(6,0)]
]
obstacles.append([(5,3.5),(5,4.5),(6,4.5),(6,3.5)])
obstacles.append([(5,5.5),(5,6.5),(6,6.5),(6,5.5)])
""""""

p_goal = (8,4)
st = time.time()
shortest_path , obstacles = global_navigation(p_robot, obstacles, p_goal, merge_need=True, visualize=False) 
print("soltution : ", shortest_path)
print("Execution time : ", time.time() - st)"""