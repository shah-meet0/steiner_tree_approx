/*************************************************************************************
 * Allocations and Algorithms
 * Graph.java
 * Authors:
 * Meet Shah - i6196781
 * Max van den Broek - i6161647
 * Class which defines a graph.
 * Characterised by a set of Nodes, each of which have an attached set of edges.
 * Required classes: Node, Edge.
 *************************************************************************************/

package approxsteinertree;

import java.util.ArrayList;
import java.util.HashMap;
import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class Graph {
    private final HashMap<Integer, Node> nodes;

    public Graph(HashMap<Integer, Node> nodes){
        this.nodes = nodes;
    }

    /**
     * Constructor which creates a graph from a particular instance file as defined in Project
     * @param filepath path to the instance file
     */
    public Graph(String filepath) {
        this.nodes = new HashMap<>();
        try {
            File instance = new File(filepath);
            Scanner input = new Scanner(instance);
            input.nextLine();
            input.next();
            int numberNodes = input.nextInt();
            for (int i = 1; i <= numberNodes; i++) {
                nodes.put(i, new Node(i));
            }
            input.next();
            int numberEdges = input.nextInt();
            for (int j = 0; j < numberEdges; j++) {
                input.next();
                int firstNode = input.nextInt();
                int secondNode = input.nextInt();
                int weight = input.nextInt();
                nodes.get(firstNode).addEdge(secondNode, weight); //add edges to both nodes.
                nodes.get(secondNode).addEdge(firstNode, weight);
            }

            for (int k = 0; k < 4; k++) {
                input.next();
            }

            int numberTerminals = input.nextInt();
            for (int t = 0; t < numberTerminals; t++) {
                input.next();
                int nodeId = input.nextInt();
                nodes.get(nodeId).setTerminal(true);
            }
            input.close();
        }

        catch (FileNotFoundException e) {
            System.out.println("Could not find file");
            System.exit(1);
        }
    }

    /**
     * Constructor used to create a graph from a set of Edges.
     * @param edgeSet Set of edges of the graph.
     * @param orig Original graph to see which nodes are terminals.
     */
    public Graph(ArrayList<Edge> edgeSet, Graph orig){
        HashMap<Integer,Node> finalGraph = new HashMap<>();
        edgeSet.forEach(edge->{
            Integer firstNode = edge.getNode1();
            Integer secondNode = edge.getNode2();
            Integer distance = edge.getCost();
            if (!finalGraph.containsKey(firstNode)) {
                finalGraph.put(firstNode, new Node(firstNode));
                if (orig.getNodes().get(firstNode).isTerminal()) {
                    finalGraph.get(firstNode).setTerminal(true);
                }
            }
            if (!finalGraph.containsKey(secondNode)) {
                finalGraph.put(secondNode, new Node(secondNode));
                if (orig.getNodes().get(secondNode).isTerminal()) {
                    finalGraph.get(secondNode).setTerminal(true);
                }
            }

            finalGraph.get(firstNode).addEdge(secondNode, distance);
            finalGraph.get(secondNode).addEdge(firstNode, distance);
        });

        this.nodes = finalGraph;
    }

    public HashMap<Integer, Node> getNodes(){
        return this.nodes;
    }

    public int numberNodes(){
        return this.nodes.size();
    }

    public void removeNode(Integer nodeId) {
        this.nodes.get(nodeId).getEdges().forEach((id,edge)-> nodes.get(id).removeEdge(nodeId));
        nodes.remove(nodeId);
    }

    /**
     * Creates a set of edges for the graph
     * @return ArrayList containing all edges of the graph.
     */
    public ArrayList <Edge> makeEdgeList(){
        ArrayList<Edge> Edges = new ArrayList<>();
        ArrayList<Edge> edgesRemoved = new ArrayList<>();
        this.getNodes().forEach((id,node)->{
            node.getEdges().forEach((id2,edge)->
            {
                Edges.add(edge);
                edgesRemoved.add(nodes.get(id2).getEdge(id));
                this.nodes.get(id2).removeEdge(id);
            });
        });
        edgesRemoved.forEach(edge-> nodes.get(edge.getNode1()).addEdge(edge.getNode2(), edge.getCost()));
        return Edges;
    }

    /**
     * Creates a set of Terminal IDs for the graph
     * @return ArrayList containing terminal ids
     */
    public ArrayList<Integer> terminalSet(){
        ArrayList<Integer> terminals = new ArrayList<>();
        this.nodes.forEach((id,node)->{
            if (node.isTerminal()){
                terminals.add(id);
            }
        });
        return terminals;
    }

    /**
     * Removes non-terminal nodes of degree 0 and 1 from a graph.
     */
    public void rmvNTDeg01(){
        ArrayList<Integer> nodesToRemove = new ArrayList<>();
        this.nodes.forEach((id,node)->
        {
            if((node.getDegree() == 1 || node.getDegree() == 0) && !node.isTerminal()){
                nodesToRemove.add(id);
            }
        });

        nodesToRemove.forEach(id ->{
            Node nodeRemoved = nodes.get(id);
            nodeRemoved.getEdges().forEach((dest,edge) -> nodes.get(dest).removeEdge(id));
            nodes.remove(id);
        });
    }

    /**
     * Returns the weight of graph
     * @param edgeList Put in new ArrayList if actual edgeList not available.
     * @return the total cost of the graph
     */
    public Integer getTotalCost (ArrayList<Edge> edgeList){
        if(edgeList.isEmpty()) {
            edgeList = this.makeEdgeList();
        }
        Integer totalCost = 0;
        for(Edge edge:edgeList){
            totalCost += edge.getCost();
        }
        return totalCost;
    }

    /**
     * Method concludes whether a given graph is a valid Steiner tree for the input graph.
     * @param original Original graph for which we are checking if this graph is a steiner tree or not
     * @return true if the graph is a valid Steiner tree, false otherwise.
     */
    public boolean isValidSteinerTree(Graph original){
        ArrayList<Integer> terminalSet = original.terminalSet();
        if(!this.isTree()){
            return false;
        }

        for(Integer key : terminalSet){
            if(!this.nodes.containsKey(key)){
                return false;
            }
        }

        for (Edge edge : this.makeEdgeList()){
            if(!original.getNodes().get(edge.getNode1()).connectsTo(edge.getNode2())){
                return false;
            }
        }
        return true;
    }

    /**
     * Method concludes whether a given graph is connected and does not have a cycle
     * @return true if the graph is a tree, false otherwise.
     */
    private boolean isTree(){
        HashMap<Integer,Boolean> visited = new HashMap<>();
        this.getNodes().forEach((id,node)->
        {
            visited.put(id,false);
        });
        Integer firstNode = (Integer) visited.keySet().toArray()[0];
        boolean isCycle = isCycle(firstNode,visited,-1);
        if(isCycle){ //checks whether there is a cycle
            return false;
        }

        if (visited.containsValue(false)){ //checks whether every node in the graph has been visited or not (connected)
            return false;
        }

        return true;
    }

    /**
     * Searches for a cycle in a graph.
     * @param currentVertex vertex currently at.
     * @param visited set of vertices which have been visited already.
     * @param parentVertex vertex from where currentVertex was arrived at.
     * @return true if there is a cycle, false otherwise.
     */
    private boolean isCycle(Integer currentVertex, HashMap<Integer,Boolean> visited, Integer parentVertex){
        visited.replace(currentVertex,true);
        for(Integer id: this.getNodes().get(currentVertex).getEdges().keySet()) {
            if(!id.equals(parentVertex)){
                if(visited.get(id)) { //if we ever go to any vertex which has been visited already, there is a cycle.
                    return true;
                }

                else {
                    if (isCycle(id, visited, currentVertex)) { //go to the next vertex and search there
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /**
     * Method which contracts two nodes of a graph. Unused in current implementation of algorithm.
     * @param node1 Node to be contracted into
     * @param node2 Node to contract
     */
    private void contractNodes (Integer node1, Integer node2){
        Node nodeToKeep = nodes.get(node1);
        Node nodeToContract = nodes.get(node2);

        //Remove node2 edges from all nodes connected to Node2, add a new edge from Node1 to that Node
        //If edge doesn't exist.
        //If edge exists, and distance from Node2 to that edge is lower, change distance for the edge in Node 1.
        nodeToContract.getEdges().forEach((dest,edge)->{
            Edge trialEdge = nodeToKeep.getEdge(dest);
            Node nodeInQuestion = nodes.get(dest);
            nodeInQuestion.removeEdge(node2);
            if(!trialEdge.equals(0,0) && trialEdge.getCost() > edge.getCost()){
                trialEdge.setCost(edge.getCost());
            }
            else {
                nodeToKeep.addEdge(dest, edge.getCost());
                nodeInQuestion.addEdge(node1, edge.getCost());
            }
        });
        if(nodeToContract.isTerminal()){
            nodeToKeep.setTerminal(true);
        }
        nodes.remove(node2);
    }

    //Method unused but has potential usages to simplify graph so that other methods are faster.
    private void cleanup(){
        ArrayList<Integer> nodesToRemove = new ArrayList<>(); //Remove nodes with degree 1 or 0
        ArrayList<Integer>  nodesToContract = new ArrayList<>(); //Contract Terminal Nodes with Degree 1
        ArrayList<Integer> nonTermDeg2 = new ArrayList<>(); //Convert non-terminal nodes with degree 2 to one edge.
        nodes.forEach((id,node) ->{
            if (node.getDegree() == 0){
                nodesToRemove.add(id);
            }
            else if(node.getDegree() == 1 && !node.isTerminal()){
                nodesToRemove.add(id);
            }
            else if (node.getDegree()==1 && node.isTerminal()){
                nodesToContract.add(id);
            }
            else if(node.getDegree()==2 && !node.isTerminal()){
                nonTermDeg2.add(id);
            }

        });

        nodesToRemove.forEach(id ->{
            Node nodeRemoved = nodes.get(id);
            nodeRemoved.getEdges().forEach((dest,edge) -> nodes.get(dest).removeEdge(id));
            nodes.remove(id);
        });

        nodesToContract.forEach(id->{
            Integer adjacentNode = (Integer) nodes.get(id).getEdges().keySet().toArray()[0];
            this.contractNodes(adjacentNode,id); //finaledges.add(this.edge);
        });

        nonTermDeg2.forEach(id->{
            if(nodes.get(id).getDegree()==2) {
                Integer neighbour1 = (Integer) nodes.get(id).getEdges().keySet().toArray()[0];
                Integer neighbour2 = (Integer) nodes.get(id).getEdges().keySet().toArray()[1];
                Node n1 = nodes.get(neighbour1);
                Node n2 = nodes.get(neighbour2);
                Integer distancexn1 = nodes.get(id).getEdge(neighbour1).getCost();
                Integer distancexn2 = nodes.get(id).getEdge(neighbour2).getCost();
                Integer totDist = distancexn1 + distancexn2;

                if (n1.connectsTo(neighbour2)) {
                    if (totDist < n1.getEdge(neighbour2).getCost()) { //if dix + diy < d(xy)
                        n1.getEdge(neighbour2).setCost(totDist);
                        n2.getEdge(neighbour1).setCost(totDist);
                    }
                } else {
                    n1.addEdge(neighbour2, totDist);
                    n2.addEdge(neighbour1, totDist);
                }
                nodes.remove(id);
                n1.removeEdge(id);
                n2.removeEdge(id);
            }
        });
    }
}
