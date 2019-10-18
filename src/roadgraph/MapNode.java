package roadgraph;

import geography.GeographicPoint;

import java.util.LinkedList;
import java.util.List;

public class MapNode {
    GeographicPoint location;
    List<MapNodeEdge> edges;

    public MapNode(GeographicPoint location) {
        this.location = location;
        edges = new LinkedList<>();
    }

    public boolean addEdge(GeographicPoint neighborLocation, String roadName, String roadType, double length) {
        if(!alreadyContainsEdge(neighborLocation)) {
            edges.add(new MapNodeEdge(neighborLocation, roadName, roadType, length));
            return true;
        }
        return false;
    }

    public List<MapNodeEdge> getEdges() {
        return edges;
    }

    private boolean alreadyContainsEdge(GeographicPoint location) {
        for(MapNodeEdge edge : edges) {
            if(edge.getLocation().equals(location)) {
                return true;
            }
        }
        return false;
    }
}
