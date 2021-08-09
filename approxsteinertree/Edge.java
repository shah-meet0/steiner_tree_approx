/*************************************************************************************
 * Allocations and Algorithms
 * Edge.java
 * Authors:
 * Meet Shah - i6196781
 * Max van den Broek - i6161647
 * Class which defines an edge between two nodes.
 * Characterised by the nodes it connects, and the distance(cost) of the edge.
 * Required classes: Node, Edge.
 *************************************************************************************/

package approxsteinertree;

public class Edge {
    private final Integer node1;
    private final Integer node2;
    private Integer cost;

    public Edge(int node1, int node2, int cost){
        this.node1 = node1;
        this.node2 = node2;
        this.cost = cost;
    }


    public Integer getNode1() {
        return node1;
    }

    public Integer getNode2() {
        return node2;
    }

    public Integer getCost() {
        return cost;
    }

    public void setCost(int cost) {
        this.cost = cost;
    }

    public boolean equals(int firstNode, int secondNode) {
        return (firstNode == this.node1 && secondNode ==this.node2 || firstNode == this.node2 && secondNode == this.node1);
    }

    public String toString(){
        return this.node1 +" to " + this.node2 + " Cost: " + this.cost;
    }

}
