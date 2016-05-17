package roadgraph;

import geography.GeographicPoint;

/**
 * 
 * @author Pooja Khullar
 *
 */

public class MapEdge {
	
	/** initialize the start and end point of the node determined in Map Node */
	private MapNode start;
	private MapNode end;
	
	/** initialize the street name */
	private String roadName;
	
	/** initialize variable to get the length of the edge/road */
	private double length;
	
	// get the node from the Map Node class
	
    MapNode getMapNode(){
    	return end;
    }
    
    // return the start point
    
    GeographicPoint getStartPoint(){
    	
    	return start.getLocation();
    }
    
    GeographicPoint getEndPoint(){
    	
    	return end.getLocation();
    }
    
    public String getRoadName(){
    	
    	return roadName;
    }
    
    double getLength(){
    	
    	return length;
    }

	 public MapNode getOtherNode(MapNode node) {
		// TODO Auto-generated method stub
		if (node.equals(start)){
			return end;
		}
		
		else if(node.equals(end)){
			return start;
	 }
	  return end;
}

	public MapNode getEndNode() {
		// TODO Auto-generated method stub
		return end;
	}
		
		
	

	
	
	
	

}
