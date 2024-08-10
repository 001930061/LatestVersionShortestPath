package algo.project;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.List;

/**
 * This class represents a negative graph. It extends the SampleGraph class
 * and provides functionality to load a graph from a CSV file, copy an existing
 * negative graph, and ensure bidirectional edges.
 */
public class SampleNegativeGraph extends SampleGraph {

    /**
     * Constructor to create a negative graph from a CSV file.
     *
     * @param filePath the file path to the CSV file.
     */
    public SampleNegativeGraph(String filePath) {
        super();  // Call the superclass (SampleGraph) constructor to initialize the graph.
        loadFromCSV(filePath);  // Load the graph data from the specified CSV file.
        ensureBidirectionalEdges();  // Ensure that each pair of vertices has bidirectional edges.
    }

    /**
     * Copy constructor to create a deep copy of a given negative graph.
     *
     * @param originalGraph the original negative graph to be copied.
     */
    public SampleNegativeGraph(SampleNegativeGraph originalGraph) {
        super();  // Call the superclass (SampleGraph) constructor to initialize the graph.

        // Copy vertices from the original graph.
        for (SampleVertex vertex : originalGraph.getAllVertices().values()) {
            this.addVertex(vertex.copy());  // Add a copy of each vertex to the new graph.
        }

        // Copy edges from the original graph.
        for (SampleEdge edge : originalGraph.getEdges()) {
            this.addEdge(edge.copy());  // Add a copy of each edge to the new graph.
        }

        // Ensure bidirectional edges in the copied graph.
        ensureBidirectionalEdges();  // Ensure that each pair of vertices has bidirectional edges.
    }

    /**
     * Method to load the graph from a CSV file. Each line in the CSV file should
     * represent an edge with format: fromVertex, toVertex, weight.
     *
     * @param filePath the file path to the CSV file.
     */
    private void loadFromCSV(String filePath) {
        try (BufferedReader br = new BufferedReader(new FileReader(filePath))) {
            String line;
            boolean isFirstLine = true;  // Flag to skip the header line if present.

            while ((line = br.readLine()) != null) {
                if (isFirstLine) {
                    isFirstLine = false;  // Skip the header line and set the flag to false.
                    continue;  // Continue to the next line.
                }

                String[] parts = line.split(",");  // Split the line by comma to get parts.
                if (parts.length == 3) {  // Ensure there are exactly three parts (from, to, weight).
                    String fromName = parts[0].trim();  // Trim and get the source vertex name.
                    String toName = parts[1].trim();  // Trim and get the destination vertex name.
                    int weight = Integer.parseInt(parts[2].trim());  // Parse the edge weight.

                    // Get or create the source vertex.
                    SampleVertex fromVertex = this.getVertexByName(fromName);
                    // Create a new vertex if not present. Latitude, Longitude, GridRow, and GridCol are 0, not useful here.
                    if (fromVertex == null) {
                        fromVertex = new SampleVertex(fromName, VertexType.PICKUP, 0, 0, 0, 0);
                        this.addVertex(fromVertex);  // Add the vertex to the graph.
                    }

                    // Get or create the destination vertex.
                    SampleVertex toVertex = this.getVertexByName(toName);
                    // Create a new vertex if not present. Latitude, Longitude, GridRow, and GridCol are 0, not useful here.
                    if (toVertex == null) {
                        toVertex = new SampleVertex(toName, VertexType.DROPOFF, 0, 0, 0, 0);
                        this.addVertex(toVertex);  // Add the vertex to the graph.
                    }

                    // Create an edge with the negated weight and add it to the graph.
                    SampleEdge edge = new SampleEdge(fromVertex, toVertex, -weight);
                    this.addEdge(edge);  // Add the edge to the graph.
                }
            }
        } catch (IOException e) {
            System.out.println("IOException occurred: " + e);  // Handle any IO exceptions that occur.
        }
    }

    /**
     * Method to ensure that each pair of vertices has bidirectional edges.
     * If an edge is missing in one direction, an edge with weight 0 is added.
     */
    private void ensureBidirectionalEdges() {
        List<SampleVertex> vertices = this.getAllVertices().values().stream().toList();  // Get all vertices in the graph.
        for (SampleVertex v1 : vertices) {  // Iterate over each vertex.
            for (SampleVertex v2 : vertices) {  // Compare with every other vertex.
                if (!v1.equals(v2)) {  // Avoid self-loops.
                    // Check if there is an edge from v1 to v2.
                    boolean hasEdgeFromV1ToV2 = this.getEdges().stream()
                            .anyMatch(edge -> edge.getVertexF().equals(v1) && edge.getVertexT().equals(v2));

                    // Check if there is an edge from v2 to v1.
                    boolean hasEdgeFromV2ToV1 = this.getEdges().stream()
                            .anyMatch(edge -> edge.getVertexF().equals(v2) && edge.getVertexT().equals(v1));

                    // If no edge exists from v1 to v2, add a bidirectional edge with weight 0.
                    if (!hasEdgeFromV1ToV2) {
                        this.addEdge(new SampleEdge(v1, v2, 0));  // Add edge from v1 to v2.
                    }
                    // If no edge exists from v2 to v1, add a bidirectional edge with weight 0.
                    if (!hasEdgeFromV2ToV1) {
                        this.addEdge(new SampleEdge(v2, v1, 0));  // Add edge from v2 to v1.
                    }
                }
            }
        }
    }

    // Method to convert custom graph to GraphStream graph for visualization
    public static org.graphstream.graph.Graph toGraphStreamGraph(SampleGraph customGraph) {
        org.graphstream.graph.Graph gsGraph = new org.graphstream.graph.implementations.SingleGraph("NegativeGraph");

        for (SampleVertex vertex : customGraph.getAllVertices().values()) {
            gsGraph.addNode(vertex.getName()).setAttribute("ui.label", vertex.getName());
        }

        int edgeCounter = 1; // Counter to ensure unique edge IDs
        for (SampleEdge edge : customGraph.getEdges()) {
            String edgeId = edge.getVertexF().getName() + "-" + edge.getVertexT().getName() + "-" + edgeCounter++;
            try {
                gsGraph.addEdge(edgeId, edge.getVertexF().getName(), edge.getVertexT().getName())
                        .setAttribute("ui.label", edge.getWeight());
            } catch (org.graphstream.graph.EdgeRejectedException e) {
                // Handle duplicate edge exceptions
                System.out.println("Duplicate edge rejected: " + edgeId);
            }
        }

        return gsGraph;
    }

    public SampleEdge getEdge(SampleVertex fromVertex, SampleVertex toVertex) {
        // Iterate through all edges in the graph
        for (SampleEdge edge : this.getEdges()) {
            // Check if the edge connects the two specified vertices
            if (edge.getVertexF().equals(fromVertex) && edge.getVertexT().equals(toVertex)) {
                return edge; // Return the edge if found
            }
        }
        // Return null if no such edge exists
        return null;
    }

}