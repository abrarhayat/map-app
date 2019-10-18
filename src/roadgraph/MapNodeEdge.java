package roadgraph;

import geography.GeographicPoint;

public class MapNodeEdge {
    private GeographicPoint location;
    private String roadName;
    private String roadType;
    private double length;

    public MapNodeEdge(GeographicPoint location) {
        this.location = location;
    }

    public MapNodeEdge(GeographicPoint location, String roadName, String roadType,
                       double length) {
        this.location = location;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public GeographicPoint getLocation() {
        return location;
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
}
