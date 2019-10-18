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

public class MapNode {
    private GeographicPoint location;
    private List<MapNodeEdge> edges;

    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new LinkedList<>();
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
            edges.add(new MapNodeEdge(endLocation, roadName, roadType, length));
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

    private boolean alreadyContainsEdge(GeographicPoint location) {
        for(MapNodeEdge edge : edges) {
            if(edge.getEndLocation().equals(location)) {
                return true;
            }
        }
        return false;
    }
}
