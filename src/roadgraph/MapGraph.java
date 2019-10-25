/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team
 * abrar hayat
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	private int numVertices;
	private int numEdges;
	//a map that maintains a map between each unique GeographicPoint and each Vertex
	private Map<GeographicPoint, MapNode> nodeMap;


	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		numVertices = 0;
		numEdges = 0;
		nodeMap = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return nodeMap.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return numEdges;
	}

	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if(location != null && !containsNodeWithLocation(location)) {
			nodeMap.put(location, new MapNode(location));
			numVertices++;
			return true;
		}
		return false;
	}

	private boolean containsNodeWithLocation(GeographicPoint location) {
		return nodeMap.containsKey(location);
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {
		if(!containsNodeWithLocation(from) || !containsNodeWithLocation(to)) {
			throw new IllegalArgumentException();
		}
		MapNode node = nodeMap.get(from);
		node.addEdge(to, roadName, roadType, length);
		numEdges ++;
	}

	private MapNode getNodeWithLocation(GeographicPoint geographicPoint) {
		if(nodeMap.get(geographicPoint) != null) {
			return nodeMap.get(geographicPoint);
		}
		throw new IllegalArgumentException("Invalid Start or Goal location entered!");
	}

	private List<GeographicPoint> trackBackPath(MapNode start, MapNode goal, Map<MapNode, MapNode> parentMap) {
		//handling case for self loop
		if(parentMap.isEmpty()) {
			parentMap.put(start, goal);
		}
		MapNode current = goal;
		LinkedList<GeographicPoint> path = new LinkedList<>();
		while (current != start) {
			path.addFirst(current.getLocation());
			//getting the previous node starting from goal and so on
			current = parentMap.get(current);
		}
		//adding the start to the beginning of the list
		path.addFirst(start.getLocation());
		System.out.println("Path: " + path);
		return path;
	}

	private double getNodeDistance(MapNode currentNode, boolean includeEndDistance) {
		if(includeEndDistance) {
			return currentNode.getDistanceFromStart() + currentNode.getDistanceFromEnd();
		}
		return currentNode.getDistanceFromStart();
	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		boolean found;
		Map<MapNode, MapNode> parentMap = new HashMap<>();

		found = bfsSearch(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap, nodeSearched);
		if(found) {
			return trackBackPath(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap);
		}
		//for cases of links not being found
		System.out.println("No links were found between the start and the goal Geographic points");
		return null;
	}

	private boolean bfsSearch(MapNode start, MapNode goal, Map<MapNode, MapNode> parentMap,
							  Consumer<GeographicPoint> nodeSearched) {
		Queue<MapNode> nodeQueue = new LinkedList<>();
		List<MapNode> visited = new ArrayList<>();
		nodeQueue.add(start);
		MapNode current;
		while (!nodeQueue.isEmpty()) {
			current = nodeQueue.remove();
			if (current.getLocation() == goal.getLocation()) {
				return true;
			}
			//System.out.println("Current Node: " + current.getLocation());
			List<MapNodeEdge> currentNeighbors = current.getEdges();
			//System.out.println("Current Neighbors: " + currentNeighbors);
			//iterator starts from the last index of the currentNeighbors list
			ListIterator<MapNodeEdge> iterator = currentNeighbors.listIterator(currentNeighbors.size());
			while (iterator.hasPrevious()) {
				GeographicPoint nextLocation = iterator.previous().getEndLocation();
				MapNode next = nodeMap.get(nextLocation);
				if(!visited.contains(next)){
					visited.add(next);
					parentMap.put(next, current);
					nodeQueue.add(next);
					//Consumer object is for the use of front-end visualization
					nodeSearched.accept(next.getLocation());
				}
			}
		}
		return false;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		boolean found;
		Map<MapNode, MapNode> parentMap = new HashMap<>();
		found = searchImpl(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap, nodeSearched,
				false);
		if(found) {
			return trackBackPath(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap);
		}
		//for cases of links not being found
		System.out.println("No links were found between the start and the goal Geographic points");
		return null;
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}

	/*
	* This method will be called by both  aStarSearch and dijkstra
	* the only difference will be whether we want to include the
	* distance of the current vertex to the goal vertex while
	* calculating the current total distance
	*
	*/
	public boolean searchImpl(MapNode start, MapNode goal, Map<MapNode, MapNode> parentMap,
							  Consumer<GeographicPoint> nodeSearched, boolean includeEndDistance) {
		PriorityQueue<MapNode> priorityQueue = new PriorityQueue<>();
		LinkedList<MapNode> visited = new LinkedList<>();
		MapNode current;
		start.setDistanceFromStart(0);
		priorityQueue.add(start);
		while (!priorityQueue.isEmpty()){
			current = priorityQueue.poll();
			//System.out.println("currentNode: " + current);
			if(!visited.contains(current)) {
				if(current.getLocation() == goal.getLocation()) {
					//System.out.println("\n" + "visited nodes: " + visited + "\n");
					//System.out.println(visited.size());
					return true;
				}
				visited.add(current);
				//Consumer object is for the use of front-end visualization
				nodeSearched.accept(current.getLocation());
				for (MapNodeEdge currentEdge : current.getEdges()) {
					MapNode currentNeighbor = nodeMap.get(currentEdge.getEndLocation());
					double currentTotalDistance = getNodeDistance(current, includeEndDistance)
							+ currentEdge.getLength();
					//for setting a parent to next relation for the first time
					parentMap.putIfAbsent(currentNeighbor, current);
					if (currentNeighbor.getDistanceFromStart() > currentTotalDistance) {
						currentNeighbor.setDistanceFromStart(currentTotalDistance);
						//update parent if distance is shorter
						parentMap.put(currentNeighbor, current);
					}
/*					System.out.println("Adding to queue: " + currentEdge.getEndLocation() +
							" with current distance total " + currentTotalDistanceFromStart + "\n");*/
					priorityQueue.add(currentNeighbor);
				}
			}
		}
		return false;
	}

	public void printMap() {
		System.out.println("\n" + "Printing Vertices and corresponding neighbors:" + "\n");
		for(GeographicPoint point : nodeMap.keySet()) {
			System.out.println("Geographic Point: " + point);
			System.out.println("Neighbors: " + nodeMap.get(point).getEdges() + "\n");
		}
	}
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		firstMap.printMap();

		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);

		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/

	}
	
}
