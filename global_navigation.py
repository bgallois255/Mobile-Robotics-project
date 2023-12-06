'''
File : global_navigation.py 
Author : Arto Dubuisson
Date : 6 déc 2023
Compute the shortest path avoiding the obstacles while reaching the goal.
'''

#======================================================================
import matplotlib.pyplot as plt
import numpy as np
import time

from matplotlib.legend_handler import HandlerTuple
from shapely.geometry import Point, Polygon, LineString, MultiPolygon
#======================================================================

IMAGE_SHAPE_X = 1920
IMAGE_SHAPE_Y = 1080

#======================================================================

def dijkstra(graph, start, goal):
    '''
    @brief   Implements the dijkstra algorithm to find the shortest path from the robot position to the goal position, only                  travelling from vertex to vertex.

    @param   graph        -> Dictionnary having entries for each vertex connected to a list of tuples containing the                                        coordinates of each vertex it can connect to, along with the length of the path to that vertex 
             start        -> Tuple of the robot position coordinates
             goal         -> Tuple of the goal position coordinates

    @return  path[::-1]   -> List of coordinates tuples indicating the successive vertices to pass by in order to go from                                    initial to goal position by travelling the shortest distance
    '''

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
    '''
    @brief   Creates the graph of the virtual environment by first generating a list of all vertices (robot, goal or obstacle                corner) in the environment.
             Then, it creates a list of all edges not intersecting with any obstacle, meaning it only contains the fully                    visible ones. 
             Finally it builds the graph.

    @param   p_robot      -> Tuple of the robot position coordinates
             obstacles    -> List of lists of organized coordinate tuples indicating the succesive corners (clockwise or                                    anticlockwise order) of the safe zones corresponding to each obstacles 
             p_goal       -> Tuple of the goal position coordinates
                 

    @return  graph        -> Dictionnary of each vertices connected to a list of tuples containing the coordinates of each                                  vertices it can connect to, along with the length of the path to that vertex
             edges        -> List of tuples of tuples: each tuple is a visible edge, containing two coordinate tuples                                        indicating its start and finsish coordinates
    '''
    
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

                intersects_obstacle = any(line.crosses(Polygon(obstacle)) or (line.within(Polygon(obstacle)) and i!=0) for                                               obstacle in obstacles if not line.touches(Polygon(obstacle)))

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
    '''
    @brief   Merges all overlapping obstacles into a single obstacle in order to reduce the amount of vertices given to the                  following algorithms and especially remove the redundant and useless ones.

    @param   obstacles                      ->  List of lists of organized coordinate tuples indicating the succesive corners                                                   (clockwise or anticlockwise order) of the safe zones corresponding to each                                                     obstacles
                 
    @return  list(polygon.exterior.coords)  -> Updated list of obstacles with possibly a shortest length, ensuring all                                                        obstacles are distinct from each other (but they can touch as long as they don't                                                overlap)
    '''
    
    polygons = [Polygon(obstacle) for obstacle in obstacles]
    
    for i in range(len(polygons)):
        if i > len(polygons) - 1 : 
            break
            
        for polygon2 in polygons:
            if polygons[i].overlaps(polygon2):
                polygons[i] = polygons[i].union(polygon2)
                polygons.remove(polygon2)
                
    return [list(polygon.exterior.coords) for polygon in polygons]


def visualize_workspace(vertices, obstacles, p_robot, p_goal, edges, graph, shortest_path=None):
    '''
    @brief   Plots the results in a virtual environment.

    @param   vertices        -> 
             obstacles       -> 
             p_robot         -> 
             p_goal          -> 
             eges            ->
             graph           ->
             shortest_path   ->

    @return  plot a graph
    '''

    plt.figure(figsize=(10, 10))
    plt.gca().set_aspect('equal', adjustable='box')
    plt.xlim(0, IMAGE_SHAPE_X)
    plt.ylim(0, IMAGE_SHAPE_Y)

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

    
    
def global_navigation(p_robot, obstacles, p_goal):
    '''
    @brief   Implements the dijkstra, visibility_graph, merge_overlapping and visualize_workspace functions to solve the global              navigation problem and visualize the solution

    @param   p_robot         -> Tuple, robot position coordinates
             obstacles       -> List of lists of organized coordinate tuples indicating the succesive corners (clockwise or                                     anticlockwise order) of the safe zones corresponding to each obstacles
             p_goal          -> Tuple, goal position coordinates
             
    @return  shortest_path   -> List of coordinates tuples indicating the successive vertices to pass by in order to go from                                   initial to goal position by travelling the shortest distance
             obstacles       -> Updated list of obstacles with possibly a shortest length, ensuring all obstacles are distinct                                 from each other (but they can touch as long as they don't overlap)
    '''
    
    obstacles = merge_overlapping_obstacles(obstacles)
        
    graph, edges = visibility_graph(p_robot, obstacles, p_goal)
    shortest_path = dijkstra(graph, p_robot, p_goal)
    
    visualize_workspace([p_robot, p_goal] + sum(obstacles, []), [Polygon(obs) for obs in obstacles],
                         p_robot, p_goal, edges, graph, shortest_path)
        
    return shortest_path, obstacles
