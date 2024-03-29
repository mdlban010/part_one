import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Scanner;

public class DijkstraAlgorithm {

    public static void dijkstra(Graph graph, Node source) {
        source.distance = 0;
        PriorityQueue<Node> queue = new PriorityQueue<>();
        queue.add(source);

        while (!queue.isEmpty()) {
            Node current = queue.poll();

            for (Edge edge : current.neighbours) {
                int distanceThroughCurrent = current.distance + edge.weight;
                if (distanceThroughCurrent < edge.destination.distance) {
                    queue.remove(edge.destination);
                    edge.destination.distance = distanceThroughCurrent;
                    edge.destination.previous = current;
                    queue.add(edge.destination);
                }
            }
        }
    }

    public static List<Node> getShortestPathTo(Node target) {
        List<Node> path = new ArrayList<>();
        for (Node node = target; node != null; node = node.previous) {
            path.add(node);
        }
        Collections.reverse(path);
        return path;
    }

    public static void main(String[] args) {
        Graph graph = createGraphFromUserInput();
        
        Scanner scanner = new Scanner(System.in);
        System.out.print("Enter source node id: ");
        int sourceId = scanner.nextInt();
        Node source = graph.getNodeById(sourceId);

        dijkstra(graph, source);

        System.out.print("Enter destination node id: ");
        int destinationId = scanner.nextInt();
        Node destination = graph.getNodeById(destinationId);

        List<Node> shortestPath = getShortestPathTo(destination);
        System.out.println("Shortest path from source to destination: " + shortestPath);
    }


public static Graph createGraphFromUserInput() {
    Graph graph = new Graph();
    Scanner scanner = new Scanner(System.in);
    
    System.out.print("Enter the number of nodes: ");
    int numNodes = scanner.nextInt();
    for (int i = 1; i <= numNodes; i++) {
        graph.addNode(new Node(i));
    }

    System.out.print("Enter the number of edges: ");
    int numEdges = scanner.nextInt();
    for (int i = 0; i < numEdges; i++) {
        System.out.print("Enter source node id for edge " + (i + 1) + ": ");
        int sourceId = scanner.nextInt();
        
        System.out.print("Enter destination node id for edge " + (i + 1) + ": ");
        int destId = scanner.nextInt();
        System.out.print("Enter weight for edge " + (i + 1) + ": ");
        int weight = scanner.nextInt();
        
        Node sourceNode = graph.getNodeById(sourceId);
        Node destNode = graph.getNodeById(destId);
        graph.addEdge(sourceNode, destNode, weight);
    }
    
    return graph;
}
}

class Graph {
    List<Node> nodes;
    public Graph() {
        nodes = new ArrayList<>();
    }

    public void addNode(Node node) {
        nodes.add(node);
    }

    public Node getNodeById(int id) {
        for (Node node : nodes) {
            if (node.id == id) {
                return node;
            }
        }
        return null;
    }
    
    public void addEdge(Node source, Node destination, int weight) {
        source.neighbours.add(new Edge(destination, weight));
    }
}

class Node implements Comparable<Node>{
    int id;
    List<Edge> neighbours;
    int distance;
    Node previous;

    public Node(int id) {
        this.id = id;
        neighbours = new ArrayList<>();
        distance = Integer.MAX_VALUE;
    }

    public int compareTo(Node other){
        return Integer.compare(this.distance, other.distance);
    }
}

class Edge {
    Node destination;
    int weight;

    public Edge(Node destination, int weight) {
        this.destination = destination;
        this.weight = weight;
    }
}
