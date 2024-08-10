package algo.project;

import java.util.ArrayList;
import java.util.List;

public class SampleBellmanFord {
    // The negative graph to run the Bellman-Ford algorithm on
    private SampleNegativeGraph negativeGraph;
    // The list to store vertices that are part of the negative cycle
    private List<SampleVertex> result;

    // Constructor to initialize the Bellman-Ford object with a negative graph
    public SampleBellmanFord(SampleNegativeGraph negativeGraph) {
        // Make a copy of the provided negative graph
        this.negativeGraph = new SampleNegativeGraph(negativeGraph);
        // Initialize the result list to store vertices involved in the negative cycle
        this.result = new ArrayList<>();
    }

    // Method to run the Bellman-Ford algorithm from a given source vertex
    public boolean run(SampleVertex source) {
        // Reinitialize distances for all vertices to positive infinity and reset parent pointers
        for (SampleVertex vertex : negativeGraph.getAllVertices().values()) {
            vertex.setDistance(Double.POSITIVE_INFINITY); // Set distance to infinity
            vertex.setParent(null); // Clear parent reference
        }

        // Set the distance of the source vertex to 0
        source.setDistance(0);

        // Perform relaxation for |V| - 1 times, where |V| is the number of vertices
        for (int i = 1; i < negativeGraph.getAllVertices().size(); i++) {
            // Iterate through all edges in the graph
            for (SampleEdge edge : negativeGraph.getEdges()) {
                SampleVertex u = edge.getVertexF(); // Get the source vertex of the edge
                SampleVertex v = edge.getVertexT(); // Get the destination vertex of the edge
                double weight = edge.getWeight(); // Get the weight of the edge

                // Relax the edge if a shorter path is found
                if (u.getDistance() != Double.POSITIVE_INFINITY && u.getDistance() + weight < v.getDistance()) {
                    v.setDistance(u.getDistance() + weight); // Update the distance to vertex v
                    v.setParent(u); // Set the parent of vertex v to vertex u
                }
            }
        }

        // Check for negative weight cycles in the graph
        boolean hasNegativeCycle = false; // Flag to indicate if a negative cycle is detected
        for (SampleEdge edge : negativeGraph.getEdges()) {
            SampleVertex u = edge.getVertexF(); // Get the source vertex of the edge
            SampleVertex v = edge.getVertexT(); // Get the destination vertex of the edge
            double weight = edge.getWeight(); // Get the weight of the edge

            // If the edge can still be relaxed, a negative cycle exists
            if (u.getDistance() != Double.POSITIVE_INFINITY && u.getDistance() + weight < v.getDistance()) {
                hasNegativeCycle = true; // Set the flag to true
                break; // Exit the loop since a negative cycle is found
            }
        }

        // Return whether a negative cycle was found
        return hasNegativeCycle;
    }

    // Method to repeatedly extract negative cycles from the graph
    public void extractNegativeCycle() {
        boolean hasNegativeCycle;

        // Repeat the process while negative cycles are found
        do {
            // Run the Bellman-Ford algorithm from a specific vertex (assumed to be the source)
            hasNegativeCycle = run(this.negativeGraph.getVertexByName("Rivian-Tenth Ave & W 15th St"));

            if (hasNegativeCycle) {
                // Find the most profitable edge in the detected negative cycle
                SampleEdge edge = getMostProfitEdge();
                if (edge != null) {
                    // Add the vertices connected by this edge to the result list
                    result.add(edge.getVertexF());
                    result.add(edge.getVertexT());
                    // Set the weight of the edge to 0, effectively removing the cycle
                    edge.setWeight(0);
                }
            }

        } while (hasNegativeCycle); // Continue until no more negative cycles are found
    }

    // Method to find the edge with the maximum profit in the graph (most negative weight)
    public SampleEdge getMostProfitEdge() {
        double maxProfit = Double.NEGATIVE_INFINITY; // Initialize max profit to the smallest possible value
        SampleEdge mostProfitEdge = null; // Initialize the most profitable edge to null

        // Iterate through all edges in the graph
        for (SampleEdge edge : this.negativeGraph.getEdges()) {
            // If the current edge's profit is greater than the current max profit
            if (-edge.getWeight() > maxProfit) {
                maxProfit = -edge.getWeight(); // Update max profit
                mostProfitEdge = edge; // Set the current edge as the most profitable edge
            }
        }

        // Return the edge with the maximum profit
        return mostProfitEdge;
    }

    // Method to get the vertices involved in the negative cycle
    public List<SampleVertex> getNegativeCycle() {
        return result; // Return the list of vertices in the negative cycle
    }
}
