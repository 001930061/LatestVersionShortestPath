package algo.project;

import java.util.ArrayList;

/**
 * This class represents a vertex in the graph.
 */
public class SampleVertex {
    // Name of the vertex, unique identifier
    private final String name;

    // Type of the vertex (e.g., normal, garage, pickup, delivery)
    private VertexType type;

    // Latitude of the vertex location
    private final double latitude;

    // Longitude of the vertex location
    private final double longitude;

    // List of edges connected to this vertex
    private ArrayList<SampleEdge> neighbors;

    // Row index in the grid where the vertex is located
    private final int mapRow;

    // Column index in the grid where the vertex is located
    private final int mapCol;

    // Status of the vertex, used in algorithms like Dijkstra's (e.g., visited/unvisited)
    private int status;

    // Distance from the source vertex, used in shortest path algorithms
    private double distance;

    // Parent vertex in shortest path calculations
    private SampleVertex parent;

    /**
     * Constructor to initialize the vertex with its name, type, latitude, longitude, grid row, and grid column.
     *
     * @param name the name of the vertex.
     * @param type the type of the vertex.
     * @param latitude the latitude of the vertex.
     * @param longitude the longitude of the vertex.
     * @param mapRow the row in the grid where the vertex is located.
     * @param mapCol the column in the grid where the vertex is located.
     */
    public SampleVertex(String name, VertexType type, double latitude, double longitude, int mapRow, int mapCol) {
        this.name = name; // Set vertex name
        this.type = type; // Set vertex type
        this.latitude = latitude; // Set latitude
        this.longitude = longitude; // Set longitude
        this.neighbors = new ArrayList<>(); // Initialize the list of neighbors
        this.mapRow = mapRow; // Set the row index in the grid
        this.mapCol = mapCol; // Set the column index in the grid

        this.status = 0; // Initialize status (default 0, e.g., unvisited)
        this.distance = 0; // Initialize distance (default 0, e.g., infinite distance)
        this.parent = null; // Initialize parent (default null, e.g., no parent)
    }

    /**
     * Returns the name of the vertex.
     *
     * @return the name of the vertex.
     */
    public String getName() {
        return name;
    }

    /**
     * Returns the type of the vertex.
     *
     * @return the type of the vertex.
     */
    public VertexType getType() {
        return type;
    }

    /**
     * Sets the type of the vertex.
     *
     * @param type the type of the vertex.
     */
    public void setType(VertexType type) {
        this.type = type;
    }

    /**
     * Returns the latitude of the vertex.
     *
     * @return the latitude of the vertex.
     */
    public double getLatitude() {
        return latitude;
    }

    /**
     * Returns the longitude of the vertex.
     *
     * @return the longitude of the vertex.
     */
    public double getLongitude() {
        return longitude;
    }

    /**
     * Returns the list of edges connected to this vertex.
     *
     * @return the list of edges connected to this vertex.
     */
    public ArrayList<SampleEdge> getNeighbors() {
        return neighbors;
    }

    /**
     * Adds a new edge to the list of neighbors.
     *
     * @param newEdge the edge to be added.
     */
    public void addNeighbor(SampleEdge newEdge) {
        neighbors.add(newEdge); // Add the edge to the neighbors list
    }

    /**
     * Returns the column index of the vertex in the grid.
     *
     * @return the column index.
     */
    public int getMapCol() {
        return mapCol;
    }

    /**
     * Returns the row index of the vertex in the grid.
     *
     * @return the row index.
     */
    public int getMapRow() {
        return mapRow;
    }

    /**
     * Returns the status of the vertex.
     *
     * @return the status of the vertex.
     */
    public int getStatus() {
        return status;
    }

    /**
     * Sets the status of the vertex.
     *
     * @param status the new status of the vertex.
     */
    public void setStatus(int status) {
        this.status = status;
    }

    /**
     * Returns the distance from the source vertex.
     *
     * @return the distance from the source vertex.
     */
    public double getDistance() {
        return distance;
    }

    /**
     * Sets the distance from the source vertex.
     *
     * @param distance the new distance from the source vertex.
     */
    public void setDistance(double distance) {
        this.distance = distance;
    }

    /**
     * Returns the parent vertex in shortest path calculations.
     *
     * @return the parent vertex.
     */
    public SampleVertex getParent() {
        return parent;
    }

    /**
     * Sets the parent vertex in shortest path calculations.
     *
     * @param parent the new parent vertex.
     */
    public void setParent(SampleVertex parent) {
        this.parent = parent;
    }

    /**
     * Creates a deep copy of the vertex, including its neighbors.
     *
     * @return a new SampleVertex instance with the same attributes and neighbors.
     */
    public SampleVertex copy() {
        SampleVertex copy = new SampleVertex(name, type, latitude, longitude, this.mapRow, this.mapCol);
        for (SampleEdge edge : neighbors) {
            copy.addNeighbor(edge.copy()); // Create a deep copy of each neighbor edge
        }

        copy.setStatus(status); // Copy the status
        copy.setDistance(distance); // Copy the distance
        copy.setParent(parent); // Copy the parent vertex

        return copy; // Return the new vertex copy
    }

    /**
     * Provides a string representation of the vertex, including its name, type, latitude, and longitude.
     *
     * @return the string representation of the vertex.
     */
    @Override
    public String toString() {
        return "Vertex(Name: " + name + ", Type: " + type + ", Lat: " + latitude + ", Long: " + longitude + ")";
    }
}