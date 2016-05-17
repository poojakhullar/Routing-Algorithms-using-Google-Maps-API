/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;
import java.util.HashSet;
import java.util.LinkedList;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, MapNode>pointNodeMap;
	private HashSet<MapEdge> edges;
	private HashSet<GeographicPoint> vertices;
	private GeographicPoint location;
	
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		pointNodeMap = new HashMap<GeographicPoint, MapNode>();
		edges = new HashSet<MapEdge>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//Implement the number of vertices
		return pointNodeMap.values().size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return vertices;
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//Implement or get number of edges
		//MapNode mn = new MapNode(location);
		//Set<MapNode> set = mn.getNeighbors();
		
		return edges.size();
	}
	
	public void printEdges() {
		System.out.println("******PRINTING EDGES******");
		System.out.println("There are " + getNumEdges() + " Edges:\n");
		for (MapEdge e : edges) {
			System.out.println(e);
		}

	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
		// get the node location from the map
		MapNode n = pointNodeMap.get(location);
		if (n == null){
			n = new MapNode(location);
			pointNodeMap.put(location, n);
			return true;
		
		}else{
			return false;
		}
		
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//1. Get the start and end point of 
		
		MapNode s = pointNodeMap.get(from);
		MapNode e = pointNodeMap.get(to);
		if (s == null)
			throw new NullPointerException("addEdge" + s);	
		if (e == null)
			throw new NullPointerException("addEdge" + e);
		
		
		
	}
	
	/** get neighbors of node */
	public Set<MapNode> getNeighbors(MapNode node){
		return node.getNeighbors();
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if(!arePreconditionsFulfilled(start, goal)){
			return null;
		}
		
		MapNode startNode = pointNodeMap.get(start);  // initialize start and end of node from the parent graph
		MapNode endNode = pointNodeMap.get(goal);
		
		HashMap<MapNode, MapNode>parentMap = new HashMap<MapNode,MapNode>(); 
		Queue<MapNode> toExplore = new LinkedList<MapNode>();   // initialize the queue
		HashSet<MapNode> visited = new HashSet<MapNode>();      // initialize visited set
		toExplore.add(startNode);       // enqueue the start of the node to the queue
		MapNode next = null;            // initialize current node
		
		while(!toExplore.isEmpty()){         // until queue is not empty remove the current node form the queue
			next = toExplore.remove();
			
			/** visualization*/
			nodeSearched.accept(next.getLocation()); // get the location of the node 
			
			if(next.equals(endNode))        // if current node is equal to the goal
				break;
			
			for (MapNode neighbor : getNeighbors(next)){    // for all the neighbors of current node
				if(!visited.contains(neighbor)){           // add the neighbors to the visited list
					visited.add(neighbor); 
					parentMap.put(neighbor, next);        // add the current node as parent to the neigbors
					toExplore.add(neighbor);   // enqueue  neighbors to the queue
					
				}
			}
			
		}

		return Path(parentMap, startNode, endNode, next.equals(endNode));
		
	}
	
	public boolean arePreconditionsFulfilled(GeographicPoint start, GeographicPoint goal) {
		if (start == null || goal == null) {
			throw new NullPointerException("Cannot find route from or to null node");
		}
		if (pointNodeMap.get(start) == null) {
			System.err.println("Start node " + start + " does not exist");
			return false;
		}
		if (pointNodeMap.get(goal) == null) {
			System.err.println("End node " + goal + " does not exist");
			return false;
		}
		return true;
	}
	

	private List<GeographicPoint> Path(HashMap<MapNode, MapNode> parentMap, MapNode start,
			MapNode end, boolean pathFound) {
		// TODO Auto-generated method stub
		if(!pathFound){
			System.out.println("No Path found");
			return null;
		}
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapNode current = end;
		
		while(!current.equals(start)){
			path.addFirst(current.getLocation());
			current = parentMap.get(current);
		}
		path.addFirst(start.getLocation());
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		if(!arePreconditionsFulfilled(start, goal)){
			return null;
		}
		
		MapNode startNode = pointNodeMap.get(start);
		MapNode endNode = pointNodeMap.get(goal);
		
		HashMap<MapNode, MapNode> parentMap = new HashMap<MapNode, MapNode>();
		PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>();
		HashSet<MapNode> visited = new HashSet<MapNode>();
		
		initializeDistances();
		
		startNode.setActualDistance(0.0);
		startNode.setDistance(0.0);
		
		toExplore.add(startNode); // enqueue the node to priority queue
		MapNode next = null;      // initialize the current node
		
		while (!toExplore.isEmpty()){
			next = toExplore.poll();   // REMOVE THE CURRENT NODE FROM THE QUEUE
			
		
		
		if(!visited.contains(next)){   // if current node is not present in the visited
			visited.add(next);         // add current to the visited
		
		
		nodeSearched.accept(next.getLocation());
		
		if(next.equals(endNode)){
			break;
		}
		
		HashMap<MapNode, Double> distanceMap = calculateDistanceMap(next);
		
		for(MapNode neighbor : getNeighbors(next)){  // for each neighbor of current node
			if(!visited.contains(neighbor)){         // add the neighbors of current node to the visited
			double distanceOfNode = next.getActualDistance() + distanceMap.get(neighbor);
			if (distanceOfNode < neighbor.getActualDistance()) {
				neighbor.setActualDistance(distanceOfNode);
				//distanceOfNode += distanceMap.add(neighbor, endNode);
				neighbor.setDistance(distanceOfNode);
				parentMap.put(neighbor, next);
				toExplore.offer(neighbor);
			}

			}
		}
	}
		}

			return Path(parentMap, startNode, endNode, endNode.equals(next));
		}
		
	
		
		

	private HashMap<MapNode, Double> calculateDistanceMap(MapNode next) {
		// TODO Auto-generated method stub
		HashMap<MapNode, Double> distancesMap = new HashMap<>();
		for(MapEdge e: next.getEdges()){
			distancesMap.put(e.getEndNode(), e.getLength());
		}
		return distancesMap;
	}

	private void initializeDistances() {
		// TODO Auto-generated method stub
		for(MapNode m : pointNodeMap.values()){
			m.setActualDistance(Double.MAX_VALUE);
			m.setDistance(Double.MAX_VALUE);
		}
		
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/san_diego.map", theMap);
		System.out.println("DONE.");
		
		System.out.println("Num nodes: " + theMap.getNumVertices());
		System.out.println("Num edges: " + theMap.getNumEdges());

		List<GeographicPoint> route = theMap.bfs(new GeographicPoint(1.0, 1.0), new GeographicPoint(8.0, -1.0));

		System.out.println(route);
		
		// You can use this method for testing.  
		
		// Use this code in Week 3 End of Week Quiz
		theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		 route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		
		
	}
	
}
