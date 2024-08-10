package algo.project;

/**
 * This class represents an edge in the graph.
 */
public class SampleEdge {

    private final SampleVertex vF;  // "From" vertex
    private final SampleVertex vT;  // "To" vertex
    private double w;  // Weight of the edge

    /**
     * Constructor initializing the edge with "From" and "To" vertices and weight.
     *
     * @param vF the "From" vertex.
     * @param vT the "To" vertex.
     * @param w the weight of the edge.
     */
    public SampleEdge(SampleVertex vF, SampleVertex vT, double w) {
        this.vF = vF;
        this.vT = vT;
        this.w = w;
    }

    /**
     * Getter for the "From" vertex.
     *
     * @return the "From" vertex.
     */
    public final SampleVertex getVertexF() {
        return vF;
    }

    /**
     * Getter for the "To" vertex.
     *
     * @return the "To" vertex.
     */
    public final SampleVertex getVertexT() {
        return vT;
    }

    /**
     * Getter for the weight of the edge.
     *
     * @return the weight of the edge.
     */
    public final double getWeight() {
        return w;
    }

    /**
     * Setter for the weight of the edge.
     *
     * @param w the weight of the edge.
     */
    public final void setWeight(double w) {
        this.w = w;
    }

    /**
     * Method to create a deep copy of the edge.
     *
     * @return a deep copy of the edge.
     */
    public SampleEdge copy() {
        return new SampleEdge(vF, vT, w); // Create a new edge with the same vertices and weight
    }

    /**
     * Method to represent the edge as a string.
     *
     * @return the string representation of the edge.
     */
    @Override
    public String toString() {
        return "Edge(From: " + vF.getName() + ", To: " + vT.getName() + ", Weight: " + w + ")"; // Format edge information
    }
}