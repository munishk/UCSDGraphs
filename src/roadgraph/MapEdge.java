package roadgraph;

import geography.GeographicPoint;

/**
 * Class representing an edge from map node in map graph.
 * @author munishk
 *
 */
public class MapEdge {
	
	private GeographicPoint source;
	
	private GeographicPoint dest;
	
	private String roadName;
	
	private String roadType;
	
	private double length;

	public MapEdge(GeographicPoint source, GeographicPoint dest, String roadName, String roadType, double length) {
		super();
		this.source = source;
		this.dest = dest;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;
	}

	public GeographicPoint getSource() {
		return source;
	}



	public GeographicPoint getDest() {
		return dest;
	}



	public String getRoadName() {
		return roadName;
	}



	public String getRoadType() {
		return roadType;
	}



	public double getLength() {
		return length;
	}



	@Override
	public String toString() {
		return "MapEdge [source=" + source + ", dest=" + dest + ", roadName=" + roadName + ", roadType=" + roadType
				+ "]";
	}
}
