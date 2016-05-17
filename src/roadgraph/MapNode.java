package roadgraph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

import geography.GeographicPoint;

/**
 * 
 * @author Pooja Khullar
 *
 */

public class MapNode {
	
	/** latitudes and longitude of the node */
	
	private GeographicPoint location;
	public GeographicPoint from;
	public GeographicPoint to;
	
	
	/** Initialize the edge of the nodes */
	
	private HashSet<MapEdge> edges;
	
	/** predicted distance */
	private double distance;
	
	/**actual distance from start to end */
	private double actualDistance;
	
	

	/** get the location of a node from the map */
	
	GeographicPoint getLocation()
	{
		return location;
	}
	
	/** List of neighbors of the node by making use of Map edge class */
	
	Set<MapEdge> getEdges(){
		
		return edges;
	}
	
	/** function to add the vertex to the map */
	
	public MapNode(GeographicPoint loc) {
		// TODO Auto-generated constructor stub
		loc = location;
		edges = new HashSet<MapEdge>(); // get the information of the neighbors of the edge
		
	}
	
	public MapNode(GeographicPoint from, GeographicPoint to){
		this.from = from;
		this.to = to;
		
	}

	public Set<MapNode> getNeighbors() {
		// TODO Auto-generated method stub
		Set<MapNode> neighbors = new HashSet<MapNode>();
		for(MapEdge edge : edges){
			neighbors.add(edge.getOtherNode(this));
		}
		return neighbors;
	}

	public double setActualDistance(double maxValue) {
		// TODO Auto-generated method stub
		return this.actualDistance = actualDistance;
		
	}

	public void setDistance(double maxValue) {
		// TODO Auto-generated method stub
		this.distance = distance;
	}

	public double getActualDistance(double actualDistance) {
		// TODO Auto-generated method stub
		return this.actualDistance;
	}
	
	public double getActualDistance() {
		return this.actualDistance;
	}
	
	
	

}
