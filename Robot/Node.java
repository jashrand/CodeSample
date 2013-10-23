

import java.util.*;

/**
 * Class representing a node in a graph
 **/
class Node implements Comparable<Node>{
    protected Point point;
    protected List<Node> neighbors = new ArrayList<Node>();
    protected float distance = Float.MAX_VALUE;
    protected boolean visited = false;
    protected Node next = null;
    
    public Node(Point p){
        point = p;
    }
    
    /**
     * Add a neighbor to this node
     **/
    public void addNeighbor(Node n){
        neighbors.add(n);
    }
    
    /**
     * Compare two nodes based on known distance to a goal node
     **/
    public int compareTo(Node n){
        if(distance == n.distance){
            return 0;
        }
        
        if(distance < n.distance){
            return -1;
        }
        return 1;
    }
    
    /**
     * Determine the distance between two nodes
     **/
    public float distance(Node n){
        return point.distance(n.point);
    }

    public String toString(){
        return this.point.toString();
    }
}
