package algo.project;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

/**
 * This class represents a positive graph with a grid-like structure.
 */
public class SamplePositiveGraph extends SampleGraph {

    public SamplePositiveGraph() {
        super(); // Initialize the parent class
    }

    /**
     * Copy constructor for deep copying SamplePositiveGraph.
     *
     * @param other the SamplePositiveGraph instance to copy.
     */
    public SamplePositiveGraph(SamplePositiveGraph other) {
        super(); // Call the default constructor of SampleGraph to initialize the new graph

        // Initialize new maps and lists for deep copying
        this.allVertices = new HashMap<>();
        this.differentVertices = new HashMap<>();
        this.differentVertices.put(VertexType.NORMAL, new ArrayList<>());
        this.differentVertices.put(VertexType.GARAGE, new ArrayList<>());
        this.differentVertices.put(VertexType.PICKUP, new ArrayList<>());
        this.differentVertices.put(VertexType.DROPOFF, new ArrayList<>());
        this.edges = new ArrayList<>();

        // Deep copy vertices
        for (SampleVertex vertex : other.getAllVertices().values()) {
            SampleVertex newVertex = vertex.copy(); // Use the copy method to create a new vertex
            this.addVertex(newVertex);
        }

        // Deep copy edges
        for (SampleEdge edge : other.getEdges()) {
            SampleEdge newEdge = edge.copy(); // Create a copy of the edge
            this.edges.add(newEdge); // Add new edge to the list
        }
    }

    /**
     * Generates a positive graph from a CSV file and creates a grid-like structure.
     *
     * @param filePath the path to the CSV file.
     */
    public void loadFromCSV(String filePath) {

        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            boolean isFirstLine = true;

            while ((line = br.readLine()) != null) {
                if (isFirstLine) {
                    isFirstLine = false;
                    continue; // Skip the header line
                }

                String[] parts = line.split(",");
                if (parts.length >= 5) { // Ensure there are enough parts in the line
                    String addressName = parts[0].trim();
                    double longitude = Double.parseDouble(parts[1].trim());
                    double latitude = Double.parseDouble(parts[2].trim());
                    int mapRow = Integer.parseInt(parts[3].trim());
                    int mapCol = Integer.parseInt(parts[4].trim());
                    VertexType type = VertexType.valueOf(parts[5].trim().toUpperCase());

                    // Create the vertex and add it to the graph
                    SampleVertex newVertex = new SampleVertex(addressName, type, latitude, longitude, mapRow, mapCol);
                    this.addVertex(newVertex);
                }
            }
        } catch (IOException e) {
            System.out.println("IOException occurred: " + e.getMessage());
        }

        // Create the grid and add edges
        createGridStructure();
    }

    /**
     * Creates a grid structure by adding edges between adjacent vertices.
     */
    private void createGridStructure() {
        Map<Integer, Map<Integer, SampleVertex>> grid = new HashMap<>();

        // Organize vertices into a grid structure based on their row and column indices
        for (SampleVertex vertex : this.getAllVertices().values()) {
            grid.computeIfAbsent(vertex.getMapRow(), k -> new HashMap<>())
                    .put(vertex.getMapCol(), vertex);
        }

        // Add edges between adjacent vertices in the grid
        for (int i = 0; i < grid.size(); i++) {
            for (int j = 0; j < grid.get(i).size(); j++) {
                SampleVertex fromVertex = grid.get(i).get(j);
                if (fromVertex == null) continue; // Skip null vertices

                // Connect to the right vertex
                if (j < grid.get(i).size() - 1) {
                    SampleVertex toVertex = grid.get(i).get(j + 1);
                    if (toVertex != null) {
                        addEdgeBetweenVertices(fromVertex, toVertex);
                    }
                }

                // Connect to the bottom vertex
                if (i < grid.size() - 1) {
                    SampleVertex toVertex = grid.get(i + 1).get(j);
                    if (toVertex != null) {
                        addEdgeBetweenVertices(fromVertex, toVertex);
                    }
                }
            }
        }

        // Debug print statement
        System.out.println("Total vertices after adding to grid: " + allVertices.values().size());
    }

    /**
     * Adds an edge between two vertices with calculated Euclidean distance as weight.
     *
     * @param fromVertex the starting vertex.
     * @param toVertex the ending vertex.
     */
    private void addEdgeBetweenVertices(SampleVertex fromVertex, SampleVertex toVertex) {
        double weight = calculateEuclideanDistance(fromVertex.getLatitude(), fromVertex.getLongitude(),
                toVertex.getLatitude(), toVertex.getLongitude());
        this.addEdge(new SampleEdge(fromVertex, toVertex, weight));
        // System.out.println("Added edge from " + fromVertex.getName() + " to " + toVertex.getName() + " with weight " + weight);
    }

    /**
     * Calculates the Euclidean distance between two points given their latitude and longitude.
     *
     * @param lat1 the latitude of the first point.
     * @param lon1 the longitude of the first point.
     * @param lat2 the latitude of the second point.
     * @param lon2 the longitude of the second point.
     * @return the formatted distance.
     */
    private double calculateEuclideanDistance(double lat1, double lon1, double lat2, double lon2) {
        double latDiff = lat2 - lat1;
        double lonDiff = lon1 - lon2;
        double distance = 100*Math.sqrt(latDiff * latDiff + lonDiff * lonDiff);
        return formatDistance(distance); // Format distance to 3 decimal places
    }

    /**
     * Formats the distance to 3 decimal places.
     *
     * @param distance the distance to format.
     * @return the formatted distance.
     */
    private double formatDistance(double distance) {
        DecimalFormat df = new DecimalFormat("#.###");
        return Double.parseDouble(df.format(distance)); // Return formatted distance
    }

    /**
     * Converts the custom graph to a GraphStream graph for visualization.
     *
     * @return the GraphStream graph.
     */
    public org.graphstream.graph.Graph toGraphStreamGraph() {
        org.graphstream.graph.Graph gsGraph = new org.graphstream.graph.implementations.SingleGraph("PositiveGraph");

        for (SampleVertex vertex : this.getAllVertices().values()) {
            gsGraph.addNode(vertex.getName()).setAttribute("ui.label", vertex.getName());
        }

        // Keep track of added edges to avoid duplicates
        Set<String> addedEdges = new HashSet<>();

        for (SampleVertex vertex : this.getAllVertices().values()) {
            for (SampleEdge edge : vertex.getNeighbors()) {
                String edgeId = edge.getVertexF().getName() + "-" + edge.getVertexT().getName();
                if (!addedEdges.contains(edgeId)) {
                    try {
                        gsGraph.addEdge(edgeId, edge.getVertexF().getName(), edge.getVertexT().getName())
                                .setAttribute("ui.label", edge.getWeight());
                        addedEdges.add(edgeId);
                    } catch (org.graphstream.graph.EdgeRejectedException e) {
                        // Handle duplicate edge exceptions
                        System.out.println("Duplicate edge rejected: " + edgeId);
                    }
                }
            }
        }

        return gsGraph;
    }

    /**
     * Prints vertices and edges of the graph.
     */
    public void printGraph() {
        System.out.println("Vertices:");
        for (SampleVertex vertex : this.getAllVertices().values()) {
            System.out.println(vertex); // Print each vertex
        }

        System.out.println("Edges:");
        for (SampleEdge edge : this.getEdges()) {
            System.out.println(edge); // Print each edge
        }
    }
}