/**
 * 
 */
package roadgraph;

import java.util.LinkedList;
import java.util.List;

import geography.GeographicPoint;

/**
 *@author chuan1900
 * A directed edge in a map graph from Node start to Node end
 */
class MapEdge 
{
	/** The name of the road */
	private String roadName;
	
	/** The type of the road */
	private String roadType;
	
	/** The two end points of the edge */
	private MapNode start;
	private MapNode end;
	
	
	/** The length of the road segment, in km */
	private double length;
	
	static final double DEFAULT_LENGTH = 0.01;
	
	
	/** Create a new MapEdge object
	 * 
	 * @param roadName
	 * @param n1  The point at one end of the segment
	 * @param n2  The point at the other end of the segment
	 * 
	 */
	MapEdge(String roadName, MapNode n1, MapNode n2) 
	{
		this(roadName, "", n1, n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new MapEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n1 The point at one end of the segment
	 * @param n2 The point at the other end of the segment
	 */
	MapEdge(String roadName, String roadType, MapNode n1, MapNode n2) 
	{
		this(roadName, roadType, n1, n2, DEFAULT_LENGTH);
	}
	
	/** 
	 * Create a new MapEdge object
	 * @param roadName  The name of the road
	 * @param roadType  The type of the road
	 * @param n1 The point at one end of the segment
	 * @param n2 The point at the other end of the segment
	 * @param length The length of the road segment
	 */	
	MapEdge(String roadName, String roadType,
			MapNode n1, MapNode n2, double length) 
	{
		this.roadName = roadName;
		start = n1;
		end = n2;
		this.roadType = roadType;
		this.length = length;
	}
	
	/**
	 * return the MapNode for the end point
	 * @return the MapNode for the end point
	 */
	MapNode getEndNode() {
	   return end;
	}
	
	/**
	 * Return the location of the start point
	 * @return The location of the start point as a GeographicPoint
	 */
	GeographicPoint getStartPoint()
	{
		return start.getLocation();
	}
	
	/**
	 * Return the location of the end point
	 * @return The location of the end point as a GeographicPoint
	 */
	GeographicPoint getEndPoint()
	{
		return end.getLocation();
	}
	
	/**
	 * Return the length of this road segment
	 * @return the length of the road segment
	 */
	double getLength()
	{
		return length;
	}
	
	/**
	 * Get the road's name
	 * @return the name of the road that this edge is on
	 */
	public String getRoadName()
	{
		return roadName;
	}

	/**
	 * Given one of the nodes involved in this edge, get the other one
	 * @param node The node on one side of this edge
	 * @return the other node involved in this edge
	 */
	MapNode getOtherNode(MapNode node)
	{
		if (node.equals(start)) 
			return end;
		else if (node.equals(end))
			return start;
		throw new IllegalArgumentException("Looking for " +
			"a point that is not in the edge");
	}
	
	/**
	 * Return a String representation for this edge.
	 */
	@Override
	public String toString()
	{
		String toReturn = "[EDGE between ";
		toReturn += "\n\t" + start.getLocation();
		toReturn += "\n\t" + end.getLocation();
		toReturn += "\nRoad name: " + roadName + " Road type: " + roadType +
				" Segment length: " + String.format("%.3g", length) + "km";
		
		return toReturn;
	}

}