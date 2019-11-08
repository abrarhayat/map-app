package roadgraph;

public enum RoadTypes {
    MOTORWAY_LINK(0), CONNECTOR(3.5), CITY_STREET(4), PRIMARY(4), SECONDARY(11),
    TERTIARY(15), LIVING_STREET(19), RESIDENTIAL(20);
    double value;

    RoadTypes(double value) {
        this.value = value;
    }

    public String toString() {
        return this.name();
    }
}
