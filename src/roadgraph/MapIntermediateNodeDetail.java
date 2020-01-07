package roadgraph;

/**
 * Map intermediate node details such as distance from source and distance to target from given node.
 * @author munishk
 *
 */
public class MapIntermediateNodeDetail {
	
	private MapNode current;
	
	private double distanceFromSource;
	
	private double distanceToDestination;
	
	private double speed;

	public MapIntermediateNodeDetail(MapNode current, double distanceFromSource, double distanceToDestination,
			double speed) {
		super();
		this.current = current;
		this.distanceFromSource = distanceFromSource;
		this.distanceToDestination = distanceToDestination;
		this.speed = speed;
	}

	public MapNode getCurrent() {
		return current;
	}

	public double getDistanceFromSource() {
		return distanceFromSource;
	}

	public double getDistanceToDestination() {
		return distanceToDestination;
	}

	public double getSpeed() {
		return speed;
	}
	
	/*
	 * Combines distance from source and distance from destination to be used in aStar search.
	 */
	public double getCombinedDistance() {
		return distanceFromSource + distanceToDestination;
	}

	@Override
	public String toString() {
		return "MapIntermediateNodeDetail [current=" + current + ", distanceFromSource=" + distanceFromSource
				+ ", distanceToDestination=" + distanceToDestination + ", speed=" + speed + "]";
	}

}
