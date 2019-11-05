package roadgraph;

public enum RoadTypes {
    MOTORWAY_LINK(2), PRIMARY(5), SECONDARY(7), TERTIARY(9), RESIDENTIAL(10);
    double value;

    RoadTypes(double value) {
        this.value = value;
    }

    public String toString() {
        return this.name();
    }
}
