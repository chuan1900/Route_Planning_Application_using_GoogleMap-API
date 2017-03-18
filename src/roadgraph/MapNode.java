package roadgraph;

import java.util.List;

import geography.GeographicPoint;

public class MapNode {
	 
	private GeographicPoint location;
	private List<MapEdge> edges;
	
	
	//constructors with parameters
	public MapNode(GeographicPoint location, List<MapEdge> edges) {
		this.location = location;
		this.edges = edges;
	}
	
		
	//set getters and setters of instance variables
	
	/**
	 * 
	 * @return location
	 */
	public GeographicPoint getLocation() {
		return location;
	}
	
	/**
	 * 
	 * @param location
	 */
	public void setLocation(GeographicPoint location) {
		this.location = location;
	}
	
	/**
	 * 
	 * @return edges
	 */
	public List<MapEdge> getEdges() {
		return edges;
	}
	
	/**
	 * 
	 * @param edges
	 */
	public void setEdges(List<MapEdge> edges) {
		this.edges = edges;
	}
	
	
}
