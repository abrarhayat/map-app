package roadgraph;

import geography.GeographicPoint;

/*
 * @author abrarhayat
 *
 * A class that represents edges between two MapNodes
 * And keeps track of its different attributes
 */


public class MapNodeEdge {
    private GeographicPoint endLocation;
    private String roadName;
    private String roadType;
    private double length;

    public MapNodeEdge(GeographicPoint location) {
        this.endLocation = location;
    }

    public MapNodeEdge(GeographicPoint location, String roadName, String roadType,
                       double length) {
        this.endLocation = location;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public GeographicPoint getEndLocation() {
        return endLocation;
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

    public String toString() {
        return endLocation.toString();
    }
}
