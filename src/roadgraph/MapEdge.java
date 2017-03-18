package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
	
	private GeographicPoint startPoint;
	private GeographicPoint endPoint;
	private String streetName;
	private String roadType;
	

	private double distance;
	
	//constructor with parameter field
	//actually in the real world, we can find streets that do not have street name
	public MapEdge(GeographicPoint startPoint, GeographicPoint endPoint, String streetName, String roadType, double distance) {
		super();
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.streetName = streetName;
		this.roadType = roadType;
		this.distance = distance;
	}
	
	/**
	 * @param startPoint
	 * @param endPoint
	 * @param distance
	 * 
	 */
	
	public MapEdge(GeographicPoint startPoint, GeographicPoint endPoint, double distance) {
		super();
		this.startPoint = startPoint;
		this.endPoint = endPoint;
		this.distance = distance;
		setStreetName("");
	}
	
	
	
	//set getters and setters of private instance variables
	
	/**
	 * 
	 * @return startPoint
	 */
	public GeographicPoint getStartPoint() {
		return startPoint;
	}
	
	
	/**
	 * 
	 * @param startPoint
	 */
	public void setStartPoint(GeographicPoint startPoint) {
		this.startPoint = startPoint;
	}
	
	
	/**
	 * 
	 * @return endPoint
	 */
	public GeographicPoint getEndPoint() {
		return endPoint;
	}
	
	/**
	 * 
	 * @param endPoint
	 */
	public void setEndPoint(GeographicPoint endPoint) {
		this.endPoint = endPoint;
	}
	
	/**
	 * 
	 * @return streetName
	 */
	public String getStreetName() {
		return streetName;
	}
	
	/**
	 * 
	 * @param streetName
	 */
	public void setStreetName(String streetName) {
		this.streetName = streetName;
	}
	
	/**
	 * 
	 * @return roadType
	 */
	public String getRoadType() {
		return roadType;
	}

	/**
	 * 
	 * @param roadType
	 */
	public void setRoadType(String roadType) {
		this.roadType = roadType;
	}
	
	/**
	 * 
	 * @return distance
	 */
	public double getDistance() {
		return distance;
	}
	
	/**
	 * 
	 * @param distance
	 */
	public void setDistance(double distance) {
		this.distance = distance;
	}
	
	
	
}
