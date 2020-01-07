package roadgraph;

import java.util.ArrayList;
import java.util.List;

import geography.GeographicPoint;

/**
 * Class representing node in map graph.
 * @author munishk
 *
 */
public class MapNode {
	
	private GeographicPoint location;
	
	private List<MapEdge> edges = new ArrayList<>();
	
	public MapNode(GeographicPoint location) {
		super();
		this.location = location;
	}

	public MapNode(GeographicPoint location, List<MapEdge> edges) {
		this.location = location;
		this.edges = edges;
	}

	public GeographicPoint getLocation() {
		return location;
	}

	public void addEdge(MapEdge edge) {
		edges.add(edge);
	}

	public List<MapEdge> getEdges() {
		return edges;
	}

	@Override
	public String toString() {
		return "MapNode [location=" + location + ", edges=" + edges + "]";
	}
}
