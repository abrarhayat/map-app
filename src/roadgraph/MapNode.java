package roadgraph;

import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

/*
* @author abrarhayat
*
* A class that represents each geographic point as a node
* And keeps track of its edges with other geographic location
*/

public class MapNode implements Comparable<MapNode> {
    private GeographicPoint location;
    private List<MapNodeEdge> edges;
    private double distanceFromStart;
    private double distanceFromEnd;
    private static final int DEFAULT_DISTANCE = 1000000000;

    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new LinkedList<>();
        distanceFromStart = DEFAULT_DISTANCE;
        distanceFromEnd = DEFAULT_DISTANCE;
    }

    public MapNode(GeographicPoint location, double distanceFromStart, double distanceFromEnd) {
        this.location = location;
        edges = new LinkedList<>();
        this.distanceFromStart = distanceFromStart;
        this.distanceFromEnd = distanceFromEnd;
    }

    /*
    * Adds an edge between this map node to the end location
    * @param endLocation the location at the end of the directed edge
    * @param roadName The name of the road
    * @param roadType The type of the road
    * @param length The length of the road, in km
    * returns true if the edge was added and false if otherwise
    */
    public boolean addEdge(GeographicPoint endLocation, String roadName, String roadType, double length) {
        if(!alreadyContainsEdge(endLocation)) {
            edges.add(new MapNodeEdge(this.location, endLocation, roadName, roadType, length));
            return true;
        }
        return false;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public List<MapNodeEdge> getEdges() {
        return edges;
    }

    public MapNodeEdge getEdgeWithEndLocation(GeographicPoint neighborEndLocation) {
        for(MapNodeEdge edge : edges) {
            if(edge.getEndLocation().equals(neighborEndLocation)) {
                return edge;
            }
        }
        return null;
    }

    public double getDistanceFromStart() {
        return distanceFromStart;
    }

    public double getDistanceFromEnd() {
        return distanceFromEnd;
    }


    public void setDistanceFromStart(double distanceFromStart) {
        this.distanceFromStart = distanceFromStart;
    }

    public void setDistanceFromEnd(double distanceFromEnd) {
        this.distanceFromEnd = distanceFromEnd;
    }

    private boolean alreadyContainsEdge(GeographicPoint location) {
        for(MapNodeEdge edge : edges) {
            if(edge.getEndLocation().equals(location)) {
                return true;
            }
        }
        return false;
    }

    public String toString() {
        return "Location: " + this.location; //+ " Neighbors:" + edges
    }

    @Override
    public int compareTo(MapNode other) {
        double otherDistance = other.getDistanceFromStart() + other.getDistanceFromEnd();
        double thisDistance = this.distanceFromStart + this.distanceFromEnd;
        if(thisDistance > otherDistance){
            return 1;
        } else if(thisDistance < otherDistance){
            return -1;
        }
        return 0;
    }
}
