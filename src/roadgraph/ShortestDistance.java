package roadgraph;

/**
 * Class containing shortest distance and previous node 
 * @author munishk
 *
 */
public class ShortestDistance {
	
	private MapNode prevNode;
	
	private double distance;

	public ShortestDistance(MapNode prevNode, double distance) {
		super();
		this.prevNode = prevNode;
		this.distance = distance;
	}

	public MapNode getPrevNode() {
		return prevNode;
	}

	public double getDistance() {
		return distance;
	}

	@Override
	public String toString() {
		return "ShortestDistance [prevNode=" + prevNode + ", distance=" + distance + "]";
	}
}
