/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {

	private Map<GeographicPoint, MapNode> nodes = new HashMap<>();

	private int edgeCount;

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return nodes.size();
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return nodes.keySet();
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return edgeCount;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		MapNode mapNode = nodes.get(location);
		if (mapNode == null) {
			mapNode = new MapNode(location);
		}
		nodes.put(location, mapNode);
		return true;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {
		MapNode mapNode = nodes.get(from);
		if (mapNode == null) {
			addVertex(from);
			mapNode = nodes.get(from);
		}

		MapEdge edge = new MapEdge(from, to, roadName, roadType, length);
		mapNode.addEdge(edge);
		edgeCount++;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		MapNode from = nodes.get(start);
		MapNode to = nodes.get(goal);

		Set<MapNode> visited = new HashSet<>();
		Queue<MapNode> queue = new LinkedList<>();
		Map<MapNode, MapNode> pathMap = new HashMap<>();
		queue.add(from);
		visited.add(from);
		boolean found = false;

		while (!queue.isEmpty()) {
			MapNode current = queue.poll();
			if (current.equals(to)) {
				found = true;
				break;
			}

			List<MapEdge> edges = current.getEdges();
			for (MapEdge edge : edges) {
				MapNode node = nodes.get(edge.getDest());
				if (!visited.contains(node)) {
					visited.add(node);
					queue.add(node);
					pathMap.put(node, current);
				}

			}

		}

		if (!found) {
			return new ArrayList<>();
		}

		LinkedList<GeographicPoint> path = new LinkedList<>();
		MapNode current = to;

		while (current != null) {
			path.addFirst(current.getLocation());
			current = pathMap.get(current);
		}
		return path;
	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		MapNode from = nodes.get(start);
		MapNode to = nodes.get(goal);

		Set<MapNode> visited = new HashSet<>();
		Queue<MapNode> queue = new LinkedList<>();
		Map<MapNode, MapNode> pathMap = new HashMap<>();
		queue.add(from);
		visited.add(from);
		boolean found = false;

		while (!queue.isEmpty()) {
			MapNode current = queue.poll();
			nodeSearched.accept(current.getLocation());
			if (current.equals(to)) {
				found = true;
				break;
			}

			List<MapEdge> edges = current.getEdges();
			for (MapEdge edge : edges) {
				MapNode node = nodes.get(edge.getDest());
				if (!visited.contains(node)) {
					visited.add(node);
					queue.add(node);
					pathMap.put(node, current);
				}

			}

		}

		if (!found) {
			return new ArrayList<>();
		}

		LinkedList<GeographicPoint> path = new LinkedList<>();
		MapNode current = to;

		while (current != null) {
			path.addFirst(current.getLocation());
			current = pathMap.get(current);
		}
		return path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// Setup priority queue based on distance from source
		Comparator<MapIntermediateNodeDetail> distanceComparator = new Comparator<MapIntermediateNodeDetail>() {
			@Override
			public int compare(MapIntermediateNodeDetail o1, MapIntermediateNodeDetail o2) {
				return (int) (o1.getDistanceFromSource() - o2.getDistanceFromSource());
			}
		};
		return shortestPath(start, goal, nodeSearched, distanceComparator);
	}
	
	private List<GeographicPoint> shortestPath(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched, Comparator<MapIntermediateNodeDetail> comparator) {
		MapNode from = nodes.get(start);
		MapNode to = nodes.get(goal);
		
        PriorityQueue<MapIntermediateNodeDetail> pq = new PriorityQueue<>(comparator);
		Set<MapNode> visited = new HashSet<>();
		Map<MapNode, ShortestDistance> shortestDistanceMap = new HashMap<>();

		pq.add(toIntermediateNodeDetail(from, from, to));
		boolean found = false;

		while (!pq.isEmpty()) {
			MapIntermediateNodeDetail current = pq.poll();
			// found destination
			if (current.getCurrent().equals(to)) {
				found = true;
				break;
			}

			// If this node is not visited then process it's edges otherwise
			// continue with next node from PQ.
			if (!visited.contains(current.getCurrent())) {

				visited.add(current.getCurrent());
				nodeSearched.accept(current.getCurrent().getLocation());

				for (MapEdge edge : current.getCurrent().getEdges()) {
					MapNode nextNode = nodes.get(edge.getDest());
					// if this node is not already visited
					if (!visited.contains(nextNode)) {
						MapIntermediateNodeDetail next = toIntermediateNodeDetail(current, nextNode, to);
						pq.add(next);

						//Update shortest distance map, if distance for next node is smaller than existing distance, then update shortest distance
						if (!shortestDistanceMap.containsKey(nextNode)
								|| shortestDistanceMap.get(nextNode).getDistance() > next.getDistanceFromSource()) {
							shortestDistanceMap.put(nextNode,
									new ShortestDistance(current.getCurrent(), next.getDistanceFromSource()));
						}
					}
				}
			}
		}

		if (!found) {
			return new ArrayList<>();
		}

		LinkedList<GeographicPoint> path = new LinkedList<>();
		MapNode current = to;
		while (current != null) {
			path.addFirst(current.getLocation());
			ShortestDistance sd = shortestDistanceMap.get(current);
			current = sd == null? null :sd.getPrevNode();
		}
		return path;
	}

	private MapIntermediateNodeDetail toIntermediateNodeDetail(MapIntermediateNodeDetail current, MapNode next,
			MapNode dest) {
		return new MapIntermediateNodeDetail(next,
				current.getDistanceFromSource() + current.getCurrent().getLocation().distance(next.getLocation()),
				next.getLocation().distance(dest.getLocation()), 60);
	}

	private MapIntermediateNodeDetail toIntermediateNodeDetail(MapNode source, MapNode current, MapNode dest) {
		return new MapIntermediateNodeDetail(current, source.getLocation().distance(current.getLocation()),
				current.getLocation().distance(dest.getLocation()), 60);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		       //comparator for combined distance so that nodes in opposite direction gets lower priority.
				Comparator<MapIntermediateNodeDetail> distanceComparator = new Comparator<MapIntermediateNodeDetail>() {
					@Override
					public int compare(MapIntermediateNodeDetail o1, MapIntermediateNodeDetail o2) {
						return (int) (o1.getCombinedDistance()  - o2.getCombinedDistance());
					}
				};
				return shortestPath(start, goal, nodeSearched, distanceComparator);
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.

		/*
		 * Here are some test cases you should try before you attempt the Week 3
		 * End of Week Quiz, EVEN IF you score 100% on the programming
		 * assignment.
		 */
		/*
		 * MapGraph simpleTestMap = new MapGraph();
		 * GraphLoader.loadRoadMap("data/testdata/simpletest.map",
		 * simpleTestMap);
		 * 
		 * GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		 * GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		 * 
		 * System.out.
		 * println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5"
		 * ); List<GeographicPoint> testroute =
		 * simpleTestMap.dijkstra(testStart,testEnd); List<GeographicPoint>
		 * testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		 * 
		 * 
		 * MapGraph testMap = new MapGraph();
		 * GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		 * 
		 * // A very simple test using real data testStart = new
		 * GeographicPoint(32.869423, -117.220917); testEnd = new
		 * GeographicPoint(32.869255, -117.216927); System.out.
		 * println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5"
		 * ); testroute = testMap.dijkstra(testStart,testEnd); testroute2 =
		 * testMap.aStarSearch(testStart,testEnd);
		 * 
		 * 
		 * // A slightly more complex test using real data testStart = new
		 * GeographicPoint(32.8674388, -117.2190213); testEnd = new
		 * GeographicPoint(32.8697828, -117.2244506); System.out.
		 * println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10"
		 * ); testroute = testMap.dijkstra(testStart,testEnd); testroute2 =
		 * testMap.aStarSearch(testStart,testEnd);
		 */

		/* Use this code in Week 3 End of Week Quiz */
		/*
		 * MapGraph theMap = new MapGraph();
		 * System.out.print("DONE. \nLoading the map...");
		 * GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		 * System.out.println("DONE.");
		 * 
		 * GeographicPoint start = new GeographicPoint(32.8648772,
		 * -117.2254046); GeographicPoint end = new GeographicPoint(32.8660691,
		 * -117.217393);
		 * 
		 * 
		 * List<GeographicPoint> route = theMap.dijkstra(start,end);
		 * List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		 * 
		 */

	}

}
