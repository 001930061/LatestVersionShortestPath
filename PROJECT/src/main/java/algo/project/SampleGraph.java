package algo.project;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

/**
 * This class represents a graph with vertices and edges.
 */
public class SampleGraph {

    Map<String, SampleVertex> allVertices;  // Map to store vertices by their names
    Map<VertexType, ArrayList<SampleVertex>> differentVertices;  // Map to store vertices by their type (e.g., normal, garage, pickup, delivery)
    ArrayList<SampleEdge> edges;  // List to store all edges in the graph

    /**
     * Constructs a new graph with empty sets of vertices and edges.
     */
    public SampleGraph() {
        allVertices = new HashMap<>();  // Initialize map to store vertices by name
        differentVertices = new HashMap<>();  // Initialize map to store vertices by type
        differentVertices.put(VertexType.NORMAL, new ArrayList<>());  // Initialize list for normal vertices
        differentVertices.put(VertexType.GARAGE, new ArrayList<>());  // Initialize list for garage vertices
        differentVertices.put(VertexType.PICKUP, new ArrayList<>());  // Initialize list for pickup vertices
        differentVertices.put(VertexType.DROPOFF, new ArrayList<>());  // Initialize list for delivery vertices

        edges = new ArrayList<>();  // Initialize list to store edges
    }

    /**
     * Adds a vertex to the graph.
     *
     * @param vertex the vertex to be added.
     */
    public void addVertex(SampleVertex vertex) {
        if (!allVertices.containsKey(vertex.getName())) {  // Check if vertex is not already present
            allVertices.put(vertex.getName(), vertex);  // Add vertex to the map
            differentVertices.get(vertex.getType()).add(vertex);  // Add vertex to the appropriate list based on type
        }
    }

    /**
     * Gets all vertices in the graph.
     *
     * @return a map of all vertices keyed by their names.
     */
    public Map<String, SampleVertex> getAllVertices() {
        return this.allVertices;
    }

    /**
     * Gets a vertex by its name.
     *
     * @param name the name of the vertex.
     * @return the vertex with the specified name, or null if not found.
     */
    public SampleVertex getVertexByName(String name) {
        return allVertices.get(name);  // Retrieve vertex by name
    }

    /**
     * Gets a list of vertices by their type.
     *
     * @param type the type of vertex.
     * @return a list of vertices of the specified type.
     */
    public ArrayList<SampleVertex> getVerticesByType(VertexType type) {
        return differentVertices.get(type);  // Retrieve list of vertices by type
    }

    /**
     * Gets a vertex by its row and column position.
     *
     * @param row the row of the vertex.
     * @param col the column of the vertex.
     * @return the vertex at the specified row and column, or null if not found.
     */
    public SampleVertex getVertexByRowAndCol(int row, int col) {
        for (SampleVertex vertex : allVertices.values()) {
            if (vertex.getMapRow() == row && vertex.getMapCol() == col) {
                return vertex;  // Return vertex at specified position
            }
        }
        return null;  // Return null if no vertex is found at the specified row and column
    }

    /**
     * Adds an edge to the graph.
     *
     * @param edge the edge to be added.
     */
    public void addEdge(SampleEdge edge) {
        if (!edges.contains(edge)) {  // Check if edge is not already present
            edges.add(edge);  // Add edge to the list
        }

        // Add edge to the "From" vertex's neighbors
        if (!edge.getVertexF().getNeighbors().contains(edge)) {
            edge.getVertexF().addNeighbor(edge);
        }

        // Add edge to the "To" vertex's neighbors
        if (!edge.getVertexT().getNeighbors().contains(edge)) {
            edge.getVertexT().addNeighbor(edge);
        }
    }

    /**
     * Gets all edges in the graph.
     *
     * @return a list of all edges.
     */
    public ArrayList<SampleEdge> getEdges() {
        return edges;  // Return the list of edges
    }

    /**
     * Creates a deep copy of the graph.
     *
     * @return a new graph that is a deep copy of this graph.
     */
    public SampleGraph copy() {
        SampleGraph copy = new SampleGraph();  // Create a new graph

        for (SampleVertex vertex : allVertices.values()) {  // Copy each vertex
            copy.addVertex(vertex.copy());
        }

        for (Map.Entry<VertexType, ArrayList<SampleVertex>> entry : differentVertices.entrySet()) {  // Copy each list of vertices by type
            ArrayList<SampleVertex> vertexList = new ArrayList<>();
            for (SampleVertex vertex : entry.getValue()) {
                vertexList.add(vertex.copy());
            }
            copy.differentVertices.put(entry.getKey(), vertexList);
        }

        for (SampleEdge edge : edges) {  // Copy each edge
            copy.addEdge(edge.copy());
        }

        return copy;  // Return the copied graph
    }

    /**
     * Represents the graph as a string.
     *
     * @return a string representation of the graph.
     */
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Graph:\n");

        // sb.append("Normal Vertices:\n");
        // for (SampleVertex vertex : allVertices.values()) {
        //     sb.append(vertex.toString()).append("\n");  // Add each vertex to the string
        // }

        sb.append("Different Type Vertices:\n");
        for (Map.Entry<VertexType, ArrayList<SampleVertex>> entry : differentVertices.entrySet()) {
            sb.append(entry.getKey()).append(":\n");
            for (SampleVertex vertex : entry.getValue()) {
                sb.append(vertex.toString()).append("\n");  // Add each vertex of a type to the string
            }
        }

        // sb.append("Edges:\n");
        // for (SampleEdge edge : edges) {
        //     sb.append(edge.toString()).append("\n");  // Add each edge to the string
        // }

        return sb.toString();  // Return the complete string representation of the graph
    }
}