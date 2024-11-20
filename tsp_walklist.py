'''
Copyright 2024 Bill Roth

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.
'''



import osmnx as ox
import networkx as nx
from itertools import permutations
import time
import matplotlib.pyplot as plt
import folium
from datetime import datetime
import os
import webbrowser as wb


#
#


def calculate_centroid(geocodes: list) -> tuple:
    """
    Calculate the centroid of a list of OSM geocodes.

    :param geocodes: List of tuples containing latitude and longitude
    :return: Tuple containing the latitude and longitude of the centroid
    """
    latitudes = [coord[0] for coord in geocodes]
    longitudes = [coord[1] for coord in geocodes]

    centroid_latitude = sum(latitudes) / len(latitudes)
    centroid_longitude = sum(longitudes) / len(longitudes)

    return centroid_latitude, centroid_longitude
#
#

def timing_decorator(func):
    def wrapper(*args, **kwargs):
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        print(
            f"Function '{func.__name__}' took {end_time - start_time} seconds")
        return result
    return wrapper


@timing_decorator
def find_shortest_path_addresses(addresses: list):

    # Geocode addresses to get their coordinates
    coords = [ox.geocode(address) for address in addresses]
    ##
    centroid = calculate_centroid(coords)
    print(f"Centroid: {centroid}")
    ##
    # Create a graph from OSM data
    G = ox.graph_from_point(coords[0], dist=1000, network_type='walk')

    # Find the nearest nodes in the graph for each coordinate
    nodes = [ox.distance.nearest_nodes(
        G, point[1], point[0]) for point in coords]

    # Create a complete graph for TSP https://networkx.org/documentation/stable/reference/generated/networkx.generators.classic.complete_graph.html

    tsp_graph = nx.complete_graph(len(nodes))
    #
    # Now iterate across all node pairs, and caleculate weight, which equals the shortest path length between the nodes
    #
    for i, u in enumerate(nodes):
        for j, v in enumerate(nodes):
            if i != j:
                tsp_graph[i][j]['weight'] = nx.shortest_path_length(G, u, v, weight='length')

    # Solve the TSP problem
    tsp_path = nx.approximation.traveling_salesman_problem(
        tsp_graph, cycle=True)

    # Map nodes back to addresses
    sorted_addresses = [addresses[i] for i in tsp_path[:-1]]

    # Output the sorted list of addresses
    print("Sorted addresses based on the shortest path:")
    for address in sorted_addresses:
        print(address)

    sorted_nodes = [nodes[i] for i in tsp_path[:-1]]

    return G, sorted_nodes, sorted_addresses

def print_nodes(G: nx.Graph, nodes: list):
    """
    Print the nodes of a graph.

    :param G: NetworkX graph
    :param nodes: List of node IDs
    """
    for node in nodes:
        print(f"Node {node}: {G.nodes[node]}")



def plot_route_on_map(G: nx.Graph, nodes: list, addresses:list):
    """
    Plot the path of a set of ordered nodes on a map using OSMnx.

    :param G: NetworkX graph
    :param route: List of ordered node IDs representing the path
    """

    # Find the shortest path between each pair of consecutive nodes
    route = []
    for i in range(len(nodes) - 1):
        segment = nx.shortest_path(G, nodes[i], nodes[i + 1], weight='length')
        route.extend(segment[:-1])  # Add all but the last node to avoid duplication

    route.append(nodes[-1])  # Add the last node

    # Extract the latitude and longitude of each node in the route
    route_coords = [(G.nodes[node]['y'], G.nodes[node]['x']) for node in route]

    # Create a Folium map centered around the first node in the route
    m = folium.Map(location=route_coords[0], zoom_start=14)

    # Add the route to the map
    folium.PolyLine(route_coords, color='blue', weight=5, opacity=0.7).add_to(m)

    # Add markers for the start and end points
    for i in range(0,len(nodes)-1):
        if(i==0):
            folium.Marker(location=route_coords[0], popup='Start\n' + addresses[i], icon=folium.Icon(color='green')).add_to(m)
        elif(i==len(nodes)-1):
            folium.Marker(location=route_coords[-1], popup='End\n' + addresses[i], icon=folium.Icon(color='red')).add_to(m)
        else:
            folium.Marker(location=route_coords[i], popup=addresses[i], icon=folium.Icon(color='cadetblue')).add_to(m)     

    # Save the map to an HTML file and display it
    filename = 'route_map.' + str(time.time()) + '.html'

    m.save(filename)

    abs_path = os.path.abspath(filename)
    
    exito = wb.open('file://' + abs_path)
    if(not exito):
        print("Error opening the browser")

    return m


def main():
    # Your main code goes here
    # List of US street addresses
    addresses = [
        "2164 Cottle AVE SAN JOSE CA 95125",
"1106 ROYCOTT WAY SAN JOSE CA 95125",
"1107 LENNON WAY SAN JOSE CA 95125",
"1117 LINCOLN CT SAN JOSE CA 95125",
"1121 LENNON WAY SAN JOSE CA 95125",
"1121 LINCOLN CT SAN JOSE CA 95125",
"1122 ROYCOTT WAY SAN JOSE CA 95125",
"1126 LINCOLN CT SAN JOSE CA 95125",
"1128 ROYCOTT WAY SAN JOSE CA 95125",
"1129 MALONE RD SAN JOSE CA 95125",
"1134 LENNON WAY SAN JOSE CA 95125",
"1140 LENNON WAY SAN JOSE CA 95125",
"1141 WILLOW VILLAGE SQ SAN JOSE CA 95125",
"1150 MALONE RD SAN JOSE CA 95125",
"1157 ROYCOTT WAY SAN JOSE CA 95125",
"1163 LENNON WAY SAN JOSE CA 95125",
"1163 ROYCOTT WAY SAN JOSE CA 95125",
"1164 ROYCOTT WAY SAN JOSE CA 95125",
"1175 ROYCOTT WAY SAN JOSE CA 95125",
"1178 MALONE RD SAN JOSE CA 95125",
"1188 MALONE RD SAN JOSE CA 95125",
"1206 MALONE RD SAN JOSE CA 95125",
"1218 ROYCOTT WAY SAN JOSE CA 95125",
"1223 ROYCOTT WAY SAN JOSE CA 95125",
"1225 MALONE RD SAN JOSE CA 95125",
"1228 SANDRA DR SAN JOSE CA 95125",
"1229 CLARK WAY SAN JOSE CA 95125",
"1229 CURTNER AVE SAN JOSE CA 95125",
"1233 MALONE RD SAN JOSE CA 95125",
"1234 ROYCOTT WAY SAN JOSE CA 95125",
"1236 HERMOSA WAY SAN JOSE CA 95125",
"1247 SANDRA DR SAN JOSE CA 95125",
"1255 CURTNER AVE SAN JOSE CA 95125",
"1260 SANDRA DR SAN JOSE CA 95125",
"1262 HERMOSA WAY SAN JOSE CA 95125",
"1265 MALONE RD SAN JOSE CA 95125",
"1268 SANDRA DR SAN JOSE CA 95125",
"1272 CLARK WAY SAN JOSE CA 95125",
"1272 ROYCOTT WAY SAN JOSE CA 95125",
"1275 SANDRA DR SAN JOSE CA 95125",
"1287 CLARK WAY SAN JOSE CA 95125",
"1298 SANDRA DR SAN JOSE CA 95125",
"1378 FULAI CT SAN JOSE CA 95125",
"1437 CURTNER AVE SAN JOSE CA 95125",
"1448 MADRONA AVE SAN JOSE CA 95125",
"1472 LESHER CT SAN JOSE CA 95125",
"1477 LESHER CT SAN JOSE CA 95125",
"1501 GEORGETTA DR SAN JOSE CA 95125",
"1910 HICKS AVE SAN JOSE CA 95125",
"1976 HICKS AVE SAN JOSE CA 95125",
"1979 CHERRY AVE SAN JOSE CA 95125",
"2011 LINCOLN AVE SAN JOSE CA 95125",
"2074 NEWPORT AVE SAN JOSE CA 95125",
"2074 NEWPORT AVE SAN JOSE CA 95125",
"2170 NEWPORT AVE SAN JOSE CA 95125"
    ]

    print("Initialized.")
    G, nodes, addresses = find_shortest_path_addresses(addresses)
    map = plot_route_on_map(G,nodes,addresses)

    pass


if __name__ == "__main__":
    main()
