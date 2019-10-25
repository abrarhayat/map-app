package roadgraph;

import geography.GeographicPoint;

/*
 * @author abrarhayat
 *
 * A class that represents edges between two MapNodes
 * And keeps track of its different attributes
 */


public class MapNodeEdge {
    private GeographicPoint startLocation;
    private GeographicPoint endLocation;
    private String roadName;
    private String roadType;
    private double length;

    public MapNodeEdge(GeographicPoint startLocation, GeographicPoint endLocation) {
        this.startLocation = startLocation;
        this.endLocation = endLocation;
    }

    public MapNodeEdge(GeographicPoint startLocation, GeographicPoint location, String roadName, String roadType,
                       double length) {
        this.startLocation = startLocation;
        this.endLocation = location;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public GeographicPoint getStartLocation() {
        return startLocation;
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

    public void setLength(double length) {
        this.length = length;
    }

    public String toString() {
        return endLocation.toString();
    }
}
