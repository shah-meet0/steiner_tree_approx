/*************************************************************************************
 * Allocations and Algorithms
 * Main.java
 * Authors:
 * Meet Shah - i6196781
 * Max van den Broek - i6161647
 * Implements Algorithms to get a 2 approximation algorithm for the Steiner Tree problem
 * Uses Approach by Kurt Mehlhorn (1988).
 * Requires classes: Node, Edge, Graph
 *************************************************************************************/

package approxsteinertree;

import java.io.BufferedWriter;
import java.io.OutputStreamWriter;
import java.io.FileWriter;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.HashMap;
import java.util.ArrayList;


public class MehlhornAlgorithm {
    private final Graph inputGraph;

    public MehlhornAlgorithm(Graph graph) {
        this.inputGraph = graph;
    } //Uses algorithms on the given graph

    /**
     * Executes the 2 approximation algorithm
     * Gets Voronoi Regions -> Constructs a Subgraph Induced by Terminals ->
     * Makes a Minimum Spanning Tree -> Changes shortest paths to edges comprising them ->
     * Makes a Minimum Spanning Tree -> Removes non-terminal vertices of degree 1
     * @param console boolean which is true when output is to be streamed to console.
     * @param filepath filepath to which output should go, if console is false.
     * @return final steiner tree approximation graph.
     */
    public Graph execute(boolean console, String filepath){
        System.out.println("Working on: " + filepath);
        VoronoiPath voronoiPair = this.voronoiDijkstra(); //Get Voronoi sets and Paths for Each Terminal
        Graph expandedIGMST = constructVoronoiGraph(voronoiPair); //Create Induced Subgraph on Terminals, Get a Minimum Spanning Tree on it
       //Then expand the minimum spanning tree to be that of the original graph.
        MehlhornAlgorithm forFinalTree = new MehlhornAlgorithm(expandedIGMST);
        ArrayList<Edge> finalTree = forFinalTree.kruskalMST(); //Create minimum spanning tree on original graph
        Graph MST = new Graph(finalTree,inputGraph); //Make edge list into a graph
        MST.rmvNTDeg01(); //Remove non-terminal nodes of degree 1
        ArrayList<Edge> mstEdgeList = MST.makeEdgeList();
        Integer totalCost = MST.getTotalCost(mstEdgeList);
        mstEdgeList.sort(Comparator.comparing(Edge::getNode1));
        System.out.println("Steiner Tree Cost: " + totalCost);

        try {
            BufferedWriter writer;
            if (console) {
                writer = new BufferedWriter(new OutputStreamWriter(System.out));
                writer.write("VALUE " + totalCost);
                for(Edge edge:mstEdgeList){
                    writer.newLine();
                    writer.write(edge.getNode1() + " " + edge.getNode2());
                }
            }
            else {
                writer = new BufferedWriter(new FileWriter(filepath));
                writer.write("VALUE " + totalCost);
                for(Edge edge:mstEdgeList){
                    writer.newLine();
                    writer.write(edge.getNode1() + " " + edge.getNode2());
                }
            }
            writer.close();

        }
        catch(java.io.IOException e) {
            System.out.println(e);
        }
        return MST;
    }


    /***************************SINGLE SOURCE DIJKSTRA TO GET VORONOI VERTICES*********************************/


    /**
     * This method uses an adapted form of Dijkstra's algorithm on the graph to obtain Voronoi regions for each terminal
     * @return A pair (denoted as VoronoiPath) of (1) shortest paths to each vertice from their closest terminal
     * And (2) a Voronoi Entry: Each voronoi entry for a non-terminal v consists of:
     * 2.1: Vertice id, 2.2: Terminal t to whose Voronoi region the v belongs,
     * 2.3: Smallest Cost of going from t to v, 2.4: Predecessor on Path to v
     */
    private VoronoiPath voronoiDijkstra(){
        int sourceId = this.addSource(); //Adds a vertex which connects to all terminals with cost zero
        Integer infinity = Integer.MAX_VALUE;
        HashMap<Integer, Integer> intermediateCosts = new HashMap<>();
        PriorityQueue<VoronoiEntry> priority = new PriorityQueue<>(); //Priority Queue makes Dijkstra more efficient
        HashMap<Integer, VoronoiEntry> smallestCosts = new HashMap<>(); //Voronoi entries are stored here when the node is selected for Dijkstra
        HashMap<Integer, Node> nodes = inputGraph.getNodes();
        HashMap<Integer,ArrayList<Edge>> paths = new HashMap<>();
        nodes.forEach((id, node) -> { //Initialising the Hashmaps
            paths.put(id,new ArrayList<>());
            intermediateCosts.put(id, infinity);
        });

        intermediateCosts.remove(sourceId);
        smallestCosts.put(sourceId, new VoronoiEntry(sourceId,sourceId,0,0));

        nodes.get(sourceId).getEdges().forEach((id,edge) -> { //putting all terminals into smallestCost hashmap,
            // as they will be initialized first anyway
            Integer destNode = id;
            intermediateCosts.remove(destNode);
            smallestCosts.put(destNode, new VoronoiEntry(destNode,destNode,0,destNode));
            nodes.get(destNode).getEdges().forEach((destNode2,edgeToNode)-> {
                Integer costToNode = edgeToNode.getCost();
                if(!smallestCosts.containsKey(destNode2)) {
                    if (costToNode < intermediateCosts.get(destNode2)) {
                        intermediateCosts.replace(destNode, costToNode); //Update cost if it can be made lower
                        priority.add(new VoronoiEntry(destNode2, destNode, costToNode,destNode)); //Make a new entry into
                        // the priority queue, with changed terminal and predecessor
                    }
                }
            });
        });

        smallestCosts.remove(sourceId); //We remove sourceId so it doesn't interfere with other methods

        while(!intermediateCosts.isEmpty()){ //Until we set smallest cost for each node
            VoronoiEntry min = priority.poll(); //Efficient way to extract minimum distance using priority queue
            if(!smallestCosts.containsKey(min.getId())){ //If we haven't already fixed the distance for min
                Integer addedNode = min.getId();
                Integer predecessor = min.getPredecessor();
                paths.replace(addedNode, new ArrayList<>(paths.get(predecessor))); //get the path to the predecessor
                paths.get(addedNode).add(inputGraph.getNodes().get(predecessor).getEdge(addedNode)); //and add the edge
                //from predecessor to current node
                smallestCosts.put(addedNode, min); //Update the entry of min to permanent
                intermediateCosts.remove(addedNode); //Hence remove it from cost matrix
                Integer smallestCostToNode = min.getCostToTerminal();
                Integer closestTerminal = min.getClosestTerminal();
                nodes.get(addedNode).getEdges().forEach((destNode, edge)-> {
                    if(!smallestCosts.containsKey(destNode)) {
                        Integer potentialCost = edge.getCost() + smallestCostToNode;
                        if (potentialCost < intermediateCosts.get(destNode)) {
                            intermediateCosts.replace(destNode, potentialCost);
                            priority.add(new VoronoiEntry(destNode,closestTerminal,potentialCost,addedNode));
                        }
                    }
                });
            }
        }
        this.inputGraph.removeNode(sourceId);
        return new VoronoiPath(smallestCosts,paths);
    }

    /**
     * Adds a vertex which connects to all terminals with cost 0.
     * @return index of that vertex
     */
    private Integer addSource() {
        Node source = new Node(inputGraph.getNodes().size()*10);
        Integer sourceId = source.getId();
        inputGraph.getNodes().forEach((id,node)->{
            if(node.isTerminal()){
                node.addEdge(sourceId,0);
                source.addEdge(id,0);
            }
        });
        inputGraph.getNodes().put(sourceId,source);
        return sourceId;
    }

    /**
     * Class helps save and extract  all the information required from VoronoiDijkstra
     */
    private static class VoronoiEntry implements Comparable<VoronoiEntry> {
        private final Integer id;
        private final Integer closestTerminal;
        private final Integer costToTerminal;
        private final Integer predecessor;

        private VoronoiEntry(Integer id, Integer closestTerminal, Integer costToTerminal, Integer predecessor){
            this.id = id;
            this.closestTerminal = closestTerminal;
            this.costToTerminal = costToTerminal;
            this.predecessor = predecessor;
        }

        public int compareTo(VoronoiEntry other){
            return this.getCostToTerminal().compareTo(other.getCostToTerminal());
        }

        private Integer getCostToTerminal(){
            return this.costToTerminal;
        }

        private Integer getId(){
            return this.id;
        }

        private Integer getClosestTerminal() {
            return closestTerminal;
        }

        private Integer getPredecessor(){
            return this.predecessor;
        }
    }

    /**
     * Class helps save results of Voronoi Dijkstra (Voronoi entry and Path of each node)
     */
    private static class VoronoiPath {
        private final HashMap<Integer, VoronoiEntry> voronoi;
        private final HashMap<Integer, ArrayList<Edge>> paths;

        private VoronoiPath(HashMap <Integer, VoronoiEntry> voronoi, HashMap<Integer, ArrayList<Edge>> paths){
            this.voronoi = voronoi;
            this.paths = paths;
        }

        private HashMap<Integer,VoronoiEntry> getVoronoi(){
            return this.voronoi;
        }

        private HashMap<Integer,ArrayList<Edge>> getPaths(){
            return this.paths;
        }
    }

    /********************************INDUCED SUBGRAPH AND EXPANSION*******************************************/


    /**
     * This method first constructs an Induced subgraph by the terminals.
     * Then forms a minimum spanning tree on it.
     * This minimum spanning tree corresponds to the minimum spanning tree of a Terminal Subgraph Metric Completion
     * Then expands each of the edges of the MST to their path in the original graph.
     * @param voronoiPair A pair consisting of a Voronoi Entry for each node, and path to it from the closest terminal
     * @return A graph which is the expanded form of the minimum spanning tree of the subgraph induced by terminals
     */
    private Graph constructVoronoiGraph(VoronoiPath voronoiPair){
        ArrayList<Edge> edges = inputGraph.makeEdgeList();
        HashMap<Integer,Node> finalCosts = new HashMap<>();
        HashMap<Integer,VoronoiEntry> voronoi = voronoiPair.getVoronoi();
        HashMap<Integer, ArrayList<Edge>> paths = voronoiPair.getPaths();
        HashMap<Integer,HashMap<Integer,Integer>> voronoiTracker = new HashMap<>();
        //Voronoi Tracker keeps track of which edge is being used to find the
        //shortest path between terminal nodes.

        inputGraph.getNodes().forEach((id,node)-> {
            if (node.isTerminal()){
                finalCosts.put(id, new Node(id));
                finalCosts.get(id).setTerminal(true);
                voronoiTracker.put(id, new HashMap<>());
            }
        });

        edges.forEach(edge->{
            Integer nodeU = edge.getNode1();
            Integer nodeV = edge.getNode2();
            Integer nodeT1 = voronoi.get(nodeU).getClosestTerminal(); //Node nodeU belongs to voronoi region of nodeT1
            Integer nodeT2 = voronoi.get(nodeV).getClosestTerminal(); //Node nodeV belongs to voronoi region of nodeT2
            if(!nodeT1.equals(nodeT2)) {
                Integer ct1u = voronoi.get(nodeU).getCostToTerminal();
                Integer ct2v = voronoi.get(nodeV).getCostToTerminal();
                Integer cuv = edge.getCost();
                Integer totCost = ct1u + cuv + ct2v;

                if (!finalCosts.get(nodeT1).getEdges().containsKey(nodeT2)) {
                    finalCosts.get(nodeT1).addEdge(nodeT2, totCost);
                    finalCosts.get(nodeT2).addEdge(nodeT1, totCost);
                    voronoiTracker.get(nodeT1).put(nodeT2, nodeU);
                    voronoiTracker.get(nodeT2).put(nodeT1, nodeV);
                }

                else {
                    if (finalCosts.get(nodeT1).getEdge(nodeT2).getCost() > totCost) { //if a edge finds a smaller cost, update
                        finalCosts.get(nodeT1).getEdge(nodeT2).setCost(totCost);
                        finalCosts.get(nodeT2).getEdge(nodeT1).setCost(totCost);
                        voronoiTracker.get(nodeT1).replace(nodeT2, nodeU);
                        voronoiTracker.get(nodeT2).replace(nodeT1, nodeV);
                    }
                }
            }
        });

        MehlhornAlgorithm spTerminalInducedSubgraph = new MehlhornAlgorithm(new Graph(finalCosts)); //make a graph using smallest costs
        ArrayList<Edge> inducedMetricCompletionMST = spTerminalInducedSubgraph.kruskalMST();
        ArrayList<Edge> finalEdges = new ArrayList<>();
        inducedMetricCompletionMST.forEach(edge->{
            Integer t1 = edge.getNode1();
            Integer t2 = edge.getNode2();
            Integer u = voronoiTracker.get(t1).get(t2);
            Integer v = voronoiTracker.get(t2).get(t1);
            finalEdges.add(inputGraph.getNodes().get(u).getEdge(v)); //add the edge u v
            finalEdges.addAll(paths.get(u)); //and the path to u
            finalEdges.addAll(paths.get(v)); //and the path to v
        });
        return new Graph(finalEdges,inputGraph);
    }


/***************************************MAKING A MINIMUM SPANNING TREE***************************************/
    /**
     * Algorithm which implements Kruskal's Minimum Spanning Tree Algorithm
     * @return list of edges of graph included in the minimum spanning tree
     */
    private ArrayList<Edge> kruskalMST(){
        PriorityQueue<Edge> edgeSet = new PriorityQueue<>(Comparator.comparing(Edge::getCost));
        ArrayList<Edge> MST = new ArrayList<>();
        inputGraph.getNodes().forEach((id,node)-> {
            node.getEdges().forEach((dest,edge) -> edgeSet.add(edge));
        });
        DisjointSet kruskal = new DisjointSet(inputGraph);
        int numberNodes = inputGraph.getNodes().size();
        while(MST.size() != numberNodes-1){
            Edge edgeToConsider = edgeSet.poll();
            int node1 = edgeToConsider.getNode1();
            int node2 = edgeToConsider.getNode2();
            if (!kruskal.findSet(node1).equals(kruskal.findSet(node2))) { //If node 1 and node 2 are connected already
                MST.add(edgeToConsider);
                kruskal.unionSets(node1,node2); //Sets become same whenever an edge is added, so all connections
                //from node 2 now connect to node 1 and vice-versa
            }
        }
        return MST;
    }

    /**
     * A disjoint set allows for Kruskal to be implemented efficiently
     * Fast methods to obtain which set nodes belong to, and making union of sets.
     */
    private static class DisjointSet {
        private final HashMap<Integer, Integer> parent;

        private DisjointSet(Graph graph) {
            HashMap<Integer, Integer> parents = new HashMap<>();
            graph.getNodes().forEach((id, node) -> parents.put(id, id));
            this.parent = parents;
        }

        private Integer findSet(Integer id) {
            if (id.equals(parent.get(id))){
                return id;
            }
            int parentHere = findSet(parent.get(id));
            parent.replace(id,parentHere); //Update nodes while finding parent so its more efficient in later calls
            return parentHere;
        }

        private void unionSets(int node1, int node2){
            node1 = findSet(node1);
            node2 = findSet(node2);
            if(node1 != node2){
                parent.replace(node2,node1);
            }
        }
    }
}
