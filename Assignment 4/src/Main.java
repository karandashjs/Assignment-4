import java.util.*;

class Vertex {
    int id;
    List<Vertex> neighbors; // Adjacent vertices
    Map<Vertex, Integer> weights; // Weight of the edge between this vertex and its neighbors

    public Vertex(int id) {
        this.id = id;
        this.neighbors = new ArrayList<>();
        this.weights = new HashMap<>();
    }

    public void addNeighbor(Vertex neighbor, int weight) {
        neighbors.add(neighbor);
        weights.put(neighbor, weight);
    }
}


public class Graph {
    private Map<Integer, Vertex> vertices;

    public Graph() {
        this.vertices = new HashMap<>();
    }

    public void addVertex(Vertex vertex) {
        vertices.put(vertex.id, vertex);
    }

    // BFS
    public List<Integer> bfs(int startId) {
        List<Integer> result = new ArrayList<>();
        if (!vertices.containsKey(startId)) {
            return result;
        }

        Set<Integer> visited = new HashSet<>();
        Queue<Vertex> queue = new LinkedList<>();

        Vertex startVertex = vertices.get(startId);
        queue.add(startVertex);
        visited.add(startVertex.id);

        while (!queue.isEmpty()) {
            Vertex currentVertex = queue.poll();
            result.add(currentVertex.id);

            for (Vertex neighbor : currentVertex.neighbors) {
                if (!visited.contains(neighbor.id)) {
                    queue.add(neighbor);
                    visited.add(neighbor.id);
                }
            }
        }

        return result;
    }

    // Dijkstra's Algorithm
    public Map<Integer, Integer> dijkstra(int startId) {
        Map<Integer, Integer> distances = new HashMap<>();
        if (!vertices.containsKey(startId)) {
            return distances;
        }

        PriorityQueue<Vertex> pq = new PriorityQueue<>(Comparator.comparingInt(distances::getOrDefault));
        Set<Integer> visited = new HashSet<>();

        Vertex startVertex = vertices.get(startId);
        distances.put(startVertex.id, 0);
        pq.add(startVertex);

        while (!pq.isEmpty()) {
            Vertex currentVertex = pq.poll();
            int currentDistance = distances.get(currentVertex.id);
            visited.add(currentVertex.id);

            for (Vertex neighbor : currentVertex.neighbors) {
                if (!visited.contains(neighbor.id)) {
                    int edgeWeight = currentVertex.weights.get(neighbor);
                    int newDistance = currentDistance + edgeWeight;

                    if (!distances.containsKey(neighbor.id) || newDistance < distances.get(neighbor.id)) {
                        distances.put(neighbor.id, newDistance);
                        pq.add(neighbor);
                    }
                }
            }
        }

        return distances;
    }

    public static void main(String[] args) {
        Graph graph = new Graph();

        // Create vertices and add them to the graph
        Vertex vertex1 = new Vertex(1);
        Vertex vertex2 = new Vertex(2);
        Vertex vertex3 = new Vertex(3);
        Vertex vertex4 = new Vertex(4);

        graph.addVertex(vertex1);
        graph.addVertex(vertex2);
        graph.addVertex(vertex3);
        graph.addVertex(vertex4);

        // Connect vertices and assign edge weights
        vertex1.addNeighbor(vertex2, 2);
        vertex1.addNeighbor(vertex3, 5);
        vertex2.addNeighbor(vertex3, 1);
        vertex3.addNeighbor(vertex4, 3);

        // Example usage of BFS and Dijkstra's algorithm
        System.out.println("BFS traversal: " + graph.bfs(1));
        System.out.println("Shortest distances from vertex 1 using Dijkstra's algorithm: " + graph.dijkstra(1));
    }
}

