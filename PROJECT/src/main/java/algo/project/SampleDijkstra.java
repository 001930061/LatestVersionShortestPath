package algo.project;

import java.util.*;

public class SampleDijkstra {

    private SamplePositiveGraph positiveGraph; // The positive graph containing all vertices

    public SampleDijkstra(SamplePositiveGraph positiveGraph) {
        this.positiveGraph = positiveGraph;
    }

    public void runDijkstra(SampleVertex source) {
        PriorityQueue<SampleVertex> priorityQueue = new PriorityQueue<>(Comparator.comparingDouble(SampleVertex::getDistance));

        // Initialize distances and status for all vertices in the positive graph
        for (SampleVertex vertex : positiveGraph.getAllVertices().values()) {
            vertex.setDistance(Double.MAX_VALUE); // Set initial distance to infinity
            vertex.setStatus(0); // Set vertex status to unvisited
            vertex.setParent(null); // No parent initially
        }
        source.setDistance(0); // Set distance for the source vertex to 0
        priorityQueue.add(source); // Add the source vertex to the priority queue

        while (!priorityQueue.isEmpty()) {
            SampleVertex u = priorityQueue.poll(); // Get the vertex with the smallest distance
            u.setStatus(1); // Mark the vertex as visited

            for (SampleEdge edge : u.getNeighbors()) {
                SampleVertex v = edge.getVertexT(); // Get the target vertex of the edge
                if (v == u) {
                    v = edge.getVertexF();
                }
                if (v.getStatus() == 1) continue; // Skip already visited vertices

                double newDist = u.getDistance() + edge.getWeight(); // Calculate the new distance
                if (newDist < v.getDistance()) {
                    priorityQueue.remove(v); // Remove the vertex from the queue to update its distance
                    v.setDistance(newDist); // Update the distance
                    v.setParent(u); // Set the parent to the current vertex
                    priorityQueue.add(v); // Re-add the vertex to the queue with the updated distance
                }
            }
        }
    }

    public List<SampleVertex> getShortestPath(SampleVertex target) {
        List<SampleVertex> path = new ArrayList<>();
        for (SampleVertex at = target; at != null; at = at.getParent()) {
            path.add(at); // Add each vertex to the path list
        }
        Collections.reverse(path); // Reverse the list to get the path from source to target
        return path;
    }

    public void printShortestPath(SampleVertex source, SampleVertex target) {
        runDijkstra(source); // Run Dijkstra from the source to calculate all shortest paths
        List<SampleVertex> path = getShortestPath(target); // Get the shortest path to the target
        if (path.isEmpty() || !path.get(0).equals(source)) {
            System.out.println("No path from " + source.getName() + " to " + target.getName()); // If no path is found
        } else {
            String pathString = String.join(", ", path.stream()
                    .map(SampleVertex::getName)
                    .toArray(String[]::new));
            System.out.println("Shortest path from " + source.getName() + " to " + target.getName() + ": " + pathString); // Print the path
        }
    }

    public SampleVertex findNearestVertex(SampleVertex from, VertexType type) {
        runDijkstra(from); // Run Dijkstra once from the starting vertex

        double minDistance = Double.MAX_VALUE; // Initialize minimum distance to infinity
        SampleVertex nearestVertex = null;

        for (SampleVertex vertex : positiveGraph.getVerticesByType(type)) {
            double distance = vertex.getDistance();
            if (distance < minDistance) {
                minDistance = distance; // Update minimum distance
                nearestVertex = vertex; // Update nearest vertex
            }
        }

        return nearestVertex; // Return the nearest vertex
    }

    public void executeNegativeCycleAndPrintPath(SampleVertex garage, List<SampleVertex> cycle) {
        // Find the corresponding vertices in the positive graph
        List<SampleVertex> positiveCycle = new ArrayList<>();
        for (SampleVertex vertex : cycle) {
            positiveCycle.add(positiveGraph.getVertexByName(vertex.getName()));
        }

        // Find the nearest pickup vertex in the positive graph
        SampleVertex nearestPickup = findNearestVertex(garage, VertexType.PICKUP);

        // Print the path from the garage to the nearest pickup
        printShortestPath(garage, nearestPickup);

        // After reaching the nearest pickup, run Dijkstra again from that pickup
        SampleVertex currentVertex = nearestPickup;
        runDijkstra(currentVertex);

        // Iterate over the cycle in the positive graph and print the shortest paths
        for (int i = 0; i < positiveCycle.size() - 1; i++) {
            if (positiveCycle.get(i).equals(nearestPickup)) continue; // Skip the current vertex
            // Run Dijkstra again from the new current vertex to find the next shortest path
            //runDijkstra(positiveCycle.get(i));
            printShortestPath(positiveCycle.get(i),positiveCycle.get(i+1));

            // Run Dijkstra again from the new current vertex to find the next shortest path
            runDijkstra(positiveCycle.get(i+1));
        }

        // Print the path from the last vertex in the cycle back to the garage
        printShortestPath(currentVertex, garage);
    }
}