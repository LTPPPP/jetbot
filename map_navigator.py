#!/usr/bin/env python3
"""
Map Navigator for JetBot
Provides pathfinding and navigation functionality using a graph-based map
"""

import json
import math
from typing import List, Dict, Tuple, Optional
import rospy


class MapNavigator:
    """
    Graph-based map navigator for JetBot autonomous navigation
    """
    
    def __init__(self, map_file_path: str):
        """
        Initialize the navigator with a map file
        
        Args:
            map_file_path (str): Path to the JSON map file
        """
        self.map_file_path = map_file_path
        self.nodes = {}
        self.edges = {}
        self.start_node = None
        self.end_node = None
        
        self.load_map()
    
    def load_map(self):
        """
        Load map from JSON file or create a default map
        """
        try:
            with open(self.map_file_path, 'r') as f:
                map_data = json.load(f)
                
            self.nodes = map_data.get('nodes', {})
            self.edges = map_data.get('edges', {})
            self.start_node = map_data.get('start_node', 'A')
            self.end_node = map_data.get('end_node', 'Z')
            
            rospy.loginfo(f"Loaded map from {self.map_file_path}")
            rospy.loginfo(f"Nodes: {list(self.nodes.keys())}")
            rospy.loginfo(f"Start: {self.start_node}, End: {self.end_node}")
            
        except FileNotFoundError:
            rospy.logwarn(f"Map file {self.map_file_path} not found. Creating default map.")
            self.create_default_map()
            self.save_map()
    
    def create_default_map(self):
        """
        Create a default simple map for testing
        """
        # Simple grid map: A-B-C
        #                  |   |
        #                  D-E-F
        self.nodes = {
            'A': {'x': 0, 'y': 0, 'type': 'intersection'},
            'B': {'x': 1, 'y': 0, 'type': 'intersection'},
            'C': {'x': 2, 'y': 0, 'type': 'intersection'},
            'D': {'x': 0, 'y': 1, 'type': 'intersection'},
            'E': {'x': 1, 'y': 1, 'type': 'intersection'},
            'F': {'x': 2, 'y': 1, 'type': 'intersection'}
        }
        
        self.edges = {
            'A': ['B', 'D'],
            'B': ['A', 'C', 'E'],
            'C': ['B', 'F'],
            'D': ['A', 'E'],
            'E': ['B', 'D', 'F'],
            'F': ['C', 'E']
        }
        
        self.start_node = 'A'
        self.end_node = 'F'
        
        rospy.loginfo("Created default map")
    
    def save_map(self):
        """
        Save current map to JSON file
        """
        map_data = {
            'nodes': self.nodes,
            'edges': self.edges,
            'start_node': self.start_node,
            'end_node': self.end_node
        }
        
        try:
            with open(self.map_file_path, 'w') as f:
                json.dump(map_data, f, indent=2)
            rospy.loginfo(f"Saved map to {self.map_file_path}")
        except Exception as e:
            rospy.logerr(f"Failed to save map: {e}")
    
    def find_path(self, start: str, end: str, banned_edges: List[Tuple[str, str]] = None) -> List[str]:
        """
        Find path from start to end using Dijkstra's algorithm
        
        Args:
            start (str): Start node ID
            end (str): End node ID
            banned_edges (List[Tuple[str, str]]): List of banned edge pairs
            
        Returns:
            List[str]: List of node IDs representing the path
        """
        if banned_edges is None:
            banned_edges = []
        
        if start not in self.nodes or end not in self.nodes:
            rospy.logerr(f"Start ({start}) or end ({end}) node not found in map")
            return []
        
        # Dijkstra's algorithm
        distances = {node: float('inf') for node in self.nodes}
        distances[start] = 0
        previous = {}
        unvisited = set(self.nodes.keys())
        
        while unvisited:
            # Find unvisited node with minimum distance
            current = min(unvisited, key=lambda node: distances[node])
            
            if distances[current] == float('inf'):
                break  # No path exists
            
            if current == end:
                break  # Found destination
            
            unvisited.remove(current)
            
            # Check neighbors
            for neighbor in self.edges.get(current, []):
                if neighbor in unvisited:
                    # Check if edge is banned
                    edge_banned = (
                        (current, neighbor) in banned_edges or 
                        (neighbor, current) in banned_edges
                    )
                    
                    if not edge_banned:
                        # Calculate distance (using Euclidean distance)
                        distance = self.calculate_distance(current, neighbor)
                        new_distance = distances[current] + distance
                        
                        if new_distance < distances[neighbor]:
                            distances[neighbor] = new_distance
                            previous[neighbor] = current
        
        # Reconstruct path
        if end not in previous and start != end:
            rospy.logwarn(f"No path found from {start} to {end}")
            return []
        
        path = []
        current = end
        while current is not None:
            path.append(current)
            current = previous.get(current)
        
        path.reverse()
        return path
    
    def calculate_distance(self, node1: str, node2: str) -> float:
        """
        Calculate Euclidean distance between two nodes
        
        Args:
            node1 (str): First node ID
            node2 (str): Second node ID
            
        Returns:
            float: Distance between nodes
        """
        if node1 not in self.nodes or node2 not in self.nodes:
            return float('inf')
        
        n1 = self.nodes[node1]
        n2 = self.nodes[node2]
        
        dx = n1['x'] - n2['x']
        dy = n1['y'] - n2['y']
        
        return math.sqrt(dx*dx + dy*dy)
    
    def get_neighbors(self, node_id: str) -> List[str]:
        """
        Get neighboring nodes of a given node
        
        Args:
            node_id (str): Node ID
            
        Returns:
            List[str]: List of neighboring node IDs
        """
        return self.edges.get(node_id, [])
    
    def get_node_position(self, node_id: str) -> Tuple[float, float]:
        """
        Get position of a node
        
        Args:
            node_id (str): Node ID
            
        Returns:
            Tuple[float, float]: (x, y) coordinates
        """
        if node_id not in self.nodes:
            return (0.0, 0.0)
        
        node = self.nodes[node_id]
        return (node['x'], node['y'])
    
    def update_node(self, node_id: str, x: float, y: float, node_type: str = 'intersection'):
        """
        Update or add a node
        
        Args:
            node_id (str): Node ID
            x (float): X coordinate
            y (float): Y coordinate
            node_type (str): Type of node
        """
        self.nodes[node_id] = {
            'x': x,
            'y': y,
            'type': node_type
        }
        
        # Initialize edges if not exists
        if node_id not in self.edges:
            self.edges[node_id] = []
    
    def add_edge(self, node1: str, node2: str):
        """
        Add bidirectional edge between two nodes
        
        Args:
            node1 (str): First node ID
            node2 (str): Second node ID
        """
        if node1 not in self.edges:
            self.edges[node1] = []
        if node2 not in self.edges:
            self.edges[node2] = []
        
        if node2 not in self.edges[node1]:
            self.edges[node1].append(node2)
        if node1 not in self.edges[node2]:
            self.edges[node2].append(node1)
    
    def remove_edge(self, node1: str, node2: str):
        """
        Remove edge between two nodes
        
        Args:
            node1 (str): First node ID
            node2 (str): Second node ID
        """
        if node1 in self.edges and node2 in self.edges[node1]:
            self.edges[node1].remove(node2)
        if node2 in self.edges and node1 in self.edges[node2]:
            self.edges[node2].remove(node1)
    
    def get_map_info(self) -> Dict:
        """
        Get complete map information
        
        Returns:
            Dict: Map data including nodes, edges, start and end points
        """
        return {
            'nodes': self.nodes,
            'edges': self.edges,
            'start_node': self.start_node,
            'end_node': self.end_node,
            'node_count': len(self.nodes),
            'edge_count': sum(len(edges) for edges in self.edges.values()) // 2
        }