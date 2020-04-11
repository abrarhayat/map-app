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
	private Map<String, List<GeographicPoint>> savedPathsDjikstra;
	private Map<String, Double> shortestPathDjikstra;
	private Map<String, List<GeographicPoint>> savedPathsAStar;
	private Map<String, Double> shortestPathAStar;
	private double lastTSPTotalDistance;


	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph() {
		numVertices = 0;
		numEdges = 0;
		nodeMap = new HashMap<>();
		savedPathsDjikstra = new HashMap<>();
		savedPathsAStar = new HashMap<>();
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
		System.out.println("Path: " + path + "\n");
		return path;
	}

	private void updateDistances(MapNode current, MapNode currentNeighbor, MapNode goal, MapNodeEdge currentEdge,
								 Map<MapNode, MapNode> parentMap, boolean includeEndDistance) {
		double calculatedDistanceFromStart = current.getDistanceFromStart() + currentEdge.getLength();
		parentMap.putIfAbsent(currentNeighbor, current);
		if(includeEndDistance) {
			double calculatedDistanceFromEnd = currentNeighbor.getLocation().distance(goal.getLocation());
			//changing infinity distance from goal to straigth line distance
			if(currentNeighbor.getDistanceFromEnd() > calculatedDistanceFromEnd) {
				currentNeighbor.setDistanceFromEnd(calculatedDistanceFromEnd);
			}
		}
		double totalDistance = currentNeighbor.getDistanceFromStart() + currentNeighbor.getDistanceFromEnd();
		if(totalDistance > calculatedDistanceFromStart + currentNeighbor.getDistanceFromEnd()) {
			currentNeighbor.setDistanceFromStart(calculatedDistanceFromStart);
			parentMap.put(currentNeighbor, current);
		}
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
		if(savedPathsDjikstra.size() > 500) {
			savedPathsDjikstra.clear();
		}
		if(savedPathsDjikstra.containsKey(start + "-->" + goal)) {
			return savedPathsDjikstra.get(start + "-->" + goal);
		}
		boolean found;
		Map<MapNode, MapNode> parentMap = new HashMap<>();
		found = searchImpl(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap, nodeSearched,
				false);
		if(found) {
			List<GeographicPoint> path = trackBackPath(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap);
			savedPathsDjikstra.putIfAbsent(start + "-->" + goal, path);
			return path;
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
		if(savedPathsDjikstra.size() > 500) {
			savedPathsDjikstra.clear();
		}
		if(savedPathsAStar.containsKey(start + "-->" + goal)) {
			return savedPathsAStar.get(start + "-->" + goal);
		}
		boolean found;
		Map<MapNode, MapNode> parentMap = new HashMap<>();
		found = searchImpl(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap, nodeSearched,
				true);
		if(found) {
			List<GeographicPoint> path = trackBackPath(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap);
			savedPathsAStar.putIfAbsent(start + "-->" + goal, path);
			return trackBackPath(getNodeWithLocation(start), getNodeWithLocation(goal), parentMap);
		}
		//for cases of links not being found
		System.out.println("No links were found between the start and the goal Geographic points");
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
				visited.add(current);
				//Consumer object is for the use of front-end visualization
				nodeSearched.accept(current.getLocation());
				//System.out.println("Current: " + current);
				if(current.getLocation() == goal.getLocation()) {
/*					for(MapNode node : visited) {
						System.out.println("\n");
						System.out.println("Visited: " + node.getLocation());
						System.out.println("Distance from start: " + node.getDistanceFromStart());
						System.out.println("Distance from end: " + node.getDistanceFromEnd());
					}*/
					if(includeEndDistance) {
						System.out.println("A* SEARCH:");
					} else {
						System.out.println("DIJKSTRA:");
					}
					System.out.println("Visited Number of nodes: " + visited.size());
					return true;
				}
				for (MapNodeEdge currentEdge : current.getEdges()) {
					MapNode currentNeighbor = nodeMap.get(currentEdge.getEndLocation());
					//update parent if distance is shorter
					updateDistances(current, currentNeighbor, goal, currentEdge, parentMap, includeEndDistance);
					priorityQueue.add(currentNeighbor);
				}
			}
		}
		return false;
	}

	/*
	* Given a start node and a set of nodes to visit
	* Gives a greedy path for a tsp problem
	*/
	private List<GeographicPoint> getTspPathUncertain(MapNode startNode, Set<MapNode> setOfNodesToVisit) {
		List<GeographicPoint> tspPath = new LinkedList<>();
		List<MapNode> checkedNodes = new LinkedList<>();
		double lowestDistance = 1000000000;
		double currentDistance = lowestDistance;
		double totalDistance = 0;
		MapNode current = startNode;
		MapNode next = null;
		tspPath.add(startNode.getLocation());
		Map<MapNode, MapNode> parentMap = new HashMap<>();
		while (!checkedNodes.containsAll(setOfNodesToVisit)) {
			for (MapNodeEdge currentEdge : current.getEdges()) {
				currentDistance = currentEdge.getEndLocation().distance(current.getLocation());
				if (currentDistance < lowestDistance) {
					lowestDistance = currentDistance;
					next = nodeMap.get(currentEdge.getEndLocation());
				}
				checkedNodes.add(nodeMap.get(currentEdge.getEndLocation()));
			}
			if(!tspPath.contains(next.getLocation())) {
				tspPath.add(next.getLocation());
			}
			totalDistance += currentDistance;
			lowestDistance = 100000000;
			current = next;
			System.out.println("next: " + next);
		}
		return tspPath;
	}


	/*
	 * Given a start node and a set of nodes to visit
	 * Gives a greedy path for a tsp problem given that the start node has edges to all the nodes to visit
	 */
	private List<GeographicPoint> getTspPathCertain(MapNode startNode, Set<MapNode> setOfNodesToVisit) {
		List<GeographicPoint> tspPath = new LinkedList<>();
		List<MapNode> visitedNodes = new LinkedList<>();
		double lowestDistance = 1000000000;
		double currentDistance = lowestDistance;
		double totalDistance = 0;
		MapNode current = startNode;
		MapNode next = null;
		tspPath.add(startNode.getLocation());
		while (visitedNodes.size() != setOfNodesToVisit.size()) {
			for (MapNode potentialNextNode : setOfNodesToVisit) {
				if(!visitedNodes.contains(potentialNextNode)){
					currentDistance = potentialNextNode.getLocation().distance(current.getLocation());
/*					System.out.print("potentialNextNode: " + potentialNextNode + " ");
					System.out.println("Distance: " + currentDistance);*/
					if (currentDistance < lowestDistance) {
						lowestDistance = currentDistance;
						next = potentialNextNode;
					}
				}
			}
			try {
				if(!tspPath.contains(next.getLocation())) {
					tspPath.add(next.getLocation());
				}
			} catch (NullPointerException ex) {
				System.out.println("No Next Nodes available!");
				return null;
			}
			totalDistance += currentDistance;
			lowestDistance = 100000000;
			current = next;
			visitedNodes.add(next);
			//System.out.println("next: " + next);
		}
		totalDistance += next.getLocation().distance(startNode.getLocation());
		tspPath.add(startNode.getLocation());
		return tspPath;
	}

	/*
	 * Given a start node and a set of nodes to visit
	 * Gives a greedy path for a tsp problem given that the start node has edges to all the nodes to visit
	 */
	private List<GeographicPoint> getTspPath(MapNode startNode, Set<MapNode> setOfNodesToVisit) {
		Consumer<GeographicPoint> temp = (x) -> {};
		List<MapNode> tspVisited = new LinkedList<>();
		MapNode currentNode;
		MapNode nextNode = null;
		double currentLowestEdgeLength = 1000000000;
		double totalDistance = 0;
		List<GeographicPoint> potentialNextPath= null;
		List<GeographicPoint> finalPath = new LinkedList<>();
		//We need to make sure that each node has a path by djikstra to every other node.
		// If it does not have have conditions,then there is no TSP path
		for(MapNode referenceNode : setOfNodesToVisit) {
			for(MapNode nextHop : setOfNodesToVisit) {
				//System.out.println("referenceNode: " + referenceNode);
				//System.out.println("nextHop: " + nextHop);
				if(!referenceNode.getLocation().equals(nextHop.getLocation())) {
					if(dijkstra(referenceNode.getLocation(), nextHop.getLocation(), temp) == null) {
						System.out.println("No TSP Path exists!");
						return null;
					}
				}
			}
		}
		//starting the journey from the startNode
		//journey ends when we have visited all the nodes
		currentNode = startNode;
		while (!tspVisited.containsAll(setOfNodesToVisit)) {
			for (MapNode potentialNextNode : setOfNodesToVisit) {
				if (!currentNode.getLocation().equals(potentialNextNode.getLocation())
						&& !tspVisited.contains(potentialNextNode)) {
					double currentDistance = getDistanceFromDjikstraPath(currentNode.getLocation(),
							potentialNextNode.getLocation(), temp);
					potentialNextPath = dijkstra(currentNode.getLocation(), potentialNextNode.getLocation(), temp);
					if (currentDistance < currentLowestEdgeLength) {
						currentLowestEdgeLength = currentDistance;
						nextNode = potentialNextNode;
					}
				}
			}
			for(GeographicPoint point : potentialNextPath) {
				tspVisited.add(nodeMap.get(point));
				//System.out.println(tspVisited);
			}
			totalDistance += currentLowestEdgeLength;
			currentLowestEdgeLength = 1000000000;
			currentNode = nextNode;
		}
		for(MapNode node: tspVisited) {
			finalPath.add(node.getLocation());
		}
		totalDistance += getDistanceFromDjikstraPath(startNode.getLocation(), nextNode.getLocation(), temp);
		finalPath.add(startNode.getLocation());
		System.out.println("totalDistance: " + totalDistance);
		lastTSPTotalDistance = totalDistance;
		return finalPath;
	}

	private List<GeographicPoint> tspImprovedBy2OptMethod(MapNode startNode, Set<MapNode> setOfNodesToVisit) {
		Consumer<GeographicPoint> temp = (x) -> {};
		List<GeographicPoint> greedyPath = getTspPath(startNode, setOfNodesToVisit);;
		List<GeographicPoint> improvedPath = new ArrayList<>();
		List<MapNode> allNodes = new ArrayList<>(setOfNodesToVisit);
		MapNode lastNode = null;
		boolean betterPathFound = false;
		int timesTried = 0;
		double totalDistance = 0;
		while (!betterPathFound && timesTried < 200) {
			totalDistance = 0;
			List<MapNode> visited = new LinkedList<>();
			Random random = new Random();
			MapNode current = startNode;
			MapNode next = null;
			visited.add(current);
			while (!visited.containsAll(allNodes)) {
				next = allNodes.get(random.nextInt(allNodes.size()));
				if(!visited.contains(next)) {
					totalDistance += getDistanceFromDjikstraPath(current.getLocation(), next.getLocation(), temp);
					improvedPath.addAll(dijkstra(current.getLocation(), next.getLocation(), temp));
					visited.add(next);
					current = next;
				}
			}
			if(totalDistance < lastTSPTotalDistance) {
				betterPathFound = true;
			}
			//System.out.println(timesTried);
			timesTried++;
			lastNode = next;
		}
		totalDistance += getDistanceFromDjikstraPath(startNode.getLocation(), lastNode.getLocation(), temp);
		improvedPath.add(startNode.getLocation());
		//System.out.println("Potenial improvedPath: " + improvedPath);
		System.out.println("totalDistance: " + totalDistance);
		System.out.println("lastTSPTotalDistance: " + lastTSPTotalDistance);
		if(totalDistance < lastTSPTotalDistance) {
			System.out.println("Improved Path: " + improvedPath);
			return improvedPath;
		}else {
			System.out.println("No better path found");
			return greedyPath;
		}
	}

	private double getDistanceFromDjikstraPath(GeographicPoint start, GeographicPoint end,
												Consumer<GeographicPoint> nodeSearched) {
		double distance = 0;
		List<GeographicPoint> path = dijkstra(start, end, nodeSearched);
		for (int index = 0; index < path.size() - 1; index++) {
			//System.out.println("Start Node: " + nodeMap.get(potentialNextPath.get(index)));
			//System.out.println("Potential Next: " + potentialNextPath.get(index + 1));
			distance += nodeMap.get(path.get(index)).getEdgeWithEndLocation(path.get(index + 1)).getLength();
		}
		return distance;
	}

	private boolean setContainsNode(MapNode node, Set<MapNode> setOfNodes) {
		for (MapNode current : setOfNodes) {
			if(current.getLocation() == node.getLocation()) {
				return true;
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
		List<GeographicPoint> simpleDijkstra = testroute = testMap.dijkstra(testStart,testEnd);
		List<GeographicPoint> simpleAStar = testroute2 = testMap.aStarSearch(testStart,testEnd);

		// A more complex test using real data
		MapGraph testMapComplex = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMapComplex);
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		List<GeographicPoint> complexDijkstra = testMapComplex.dijkstra(testStart,testEnd);
		List<GeographicPoint> complexAStar = testMapComplex.aStarSearch(testStart,testEnd);

		
		/* Use this code in Week 3 End of Week Quiz */
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		/*
		* Test For TSP Problem
		*/
		Set<MapNode> nodesToVisit = new HashSet<MapNode>();
		int count = 0;
		MapNode startNode = null;
		for(GeographicPoint current : theMap.nodeMap.keySet()){
			startNode = theMap.nodeMap.get(current);
			break;
		}

		for (MapNodeEdge currentEdge : startNode.getEdges()) {
			System.out.println("Potential Next: " + currentEdge.getEndLocation());
			System.out.println("Current Distance: " + startNode.getLocation().distance(currentEdge.getEndLocation()));
		}
		System.out.println("\n");
		MapNode current = startNode;
		while (nodesToVisit.size() < 5) {
			for (MapNodeEdge currentEdge : current.getEdges()) {
				if(count == 0) {
					startNode = theMap.nodeMap.get(currentEdge.getEndLocation());
				}
				nodesToVisit.add(theMap.nodeMap.get(currentEdge.getEndLocation()));
				current = theMap.nodeMap.get(currentEdge.getEndLocation());
				count++;
			}
		}

		System.out.println("Start Node: " + startNode);
		System.out.println("Nodes to visit: " + nodesToVisit);
		//System.out.println("Path1: " + theMap.getTspPath(startNode, nodesToVisit));
		System.out.println("TSP Path: " + theMap.getTspPath(startNode, nodesToVisit));
		theMap.tspImprovedBy2OptMethod(startNode, nodesToVisit);
	}
}
