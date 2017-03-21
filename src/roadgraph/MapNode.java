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
	
	/** Returns whether two nodes are equal.
	 * Nodes are considered equal if their locations are the same, 
	 * even if their street list is different.
	 * @param o the node to compare to
	 * @return true if these nodes are at the same location, false otherwise
	 */
	@Override
	public boolean equals(Object o)
	{
		if (!(o instanceof MapNode) || (o == null)) {
			return false;
		}
		MapNode node = (MapNode)o;
		return node.location.equals(this.location);
	}
	
	/** Because we compare nodes using their location, we also 
	 * may use their location for HashCode.
	 * @return The HashCode for this node, which is the HashCode for the 
	 * underlying point
	 */
	@Override
	public int hashCode()
	{
		return location.hashCode();
	}
	
	/** ToString to print out a MapNode object
	 *  @return the string representation of a MapNode
	 */
	@Override
	public String toString()
	{
		String toReturn = "[NODE at location (" + location + ")";
		toReturn += " intersects streets: ";
		for (MapEdge e: edges) {
			toReturn += e.getStreetName() + ", ";
		}
		toReturn += "]";
		return toReturn;
	}

	// For debugging, output roadNames as a String.
	public String roadNamesAsString()
	{
		String toReturn = "(";
		for (MapEdge e: edges) {
			toReturn += e.getStreetName() + ", ";
		}
		toReturn += ")";
		return toReturn;
	}
	
}
