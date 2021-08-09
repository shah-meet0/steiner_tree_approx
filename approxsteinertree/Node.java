/*************************************************************************************
 * Allocations and Algorithms
 * Node.java
 * Authors:
 * Meet Shah - i6196781
 * Max van den Broek - i6161647
 * Class which defines a node of a graph. Suited for applications to Steiner Trees.
 * Characterised by an id, set of edges, and whether or not it is a Steiner Terminal
 * Required classes: Edge
 *************************************************************************************/

package approxsteinertree;

import java.util.HashMap;

public class Node {
    private final Integer id;
    private final HashMap<Integer, Edge> edges; //the key represents the id of the adjacent node, the value gives the edge.
    private boolean isTerminal;

    public Node(Integer id){
        this.id = id;
        this.edges = new HashMap<>();
        this.isTerminal = false;
    }


    public int getId(){
        return this.id;
    }

    public HashMap<Integer, Edge> getEdges(){
        return this.edges;
    }

    public boolean isTerminal() {
        return isTerminal;
    }

    public void setTerminal(boolean terminal){
        this.isTerminal = terminal;
    }

    /**
     * Function to get the degree of a node
     * @return count of nodes connected to given node
     */
    public int getDegree(){
        return this.edges.size();
    }

    /**
     * Method which adds an edge to a node
     * @param destination Integer which denotes which node the edge is towards.
     * @param weight Integer which gives weight of the edge
     */
    public void addEdge(Integer destination, Integer weight) {
        this.edges.put(destination, new Edge(this.id, destination, weight));
    }

    /**
     * Method which removes an edge from a node
     * @param destination Integer which denotes which node the removed edge is towards.
     */
    public void removeEdge(int destination) {
        this.edges.remove(destination);
    }

    /**
     * Method which tells if the node is connected to another node directly.
     * @param otherNode node to check for connection.
     * @return true if edge(thisNode,otherNode) exists, false otherwise.
     */
    public boolean connectsTo(Integer otherNode){
        return this.edges.containsKey(otherNode);
    }

    /**
     * Method to get edge between two nodes.
     * @param otherNode node to which the edge goes from this node.
     * @return edge(thisNode,otherNode) if it exists, a zero edge otherwise.
     */
    public Edge getEdge(Integer otherNode) {
        return this.edges.getOrDefault(otherNode, new Edge(0, 0, 0));
    }

    public String toString(){
        String toPrint = "Node ID:" + this.id.toString() + System.lineSeparator();
        toPrint = toPrint.concat("Is Terminal" + this.isTerminal()+System.lineSeparator());
        toPrint = toPrint.concat("Edges:" + System.lineSeparator());

        for(Edge edge: this.getEdges().values()){
            toPrint = toPrint.concat(edge.toString() + System.lineSeparator());
        }
        return toPrint;
    }




}

