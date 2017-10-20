import java.util.ArrayList;
import java.util.List;
import java.awt.Point;

public class Node {

    private Point state;
    private Node parent;
    //private Object action;
    private int pathCost;

    public Node(Point state) {
        this.state = state;
        this.pathCost = 0;
    }

    public Node(Point state, Node parent, int stepCost) {
        this(state);
        this.parent = parent;
        this.pathCost = parent.pathCost + stepCost;
    }

    public Point getState() {
        return state;
    }

    public Node getParent() {
        return parent;
    }

    public int getPathCost() {
        return pathCost;
    }

    public boolean isRootNode() {
        return parent == null;
    }

    public List<Node> getPathFromRoot() {
        List<Node> path = new ArrayList<Node>();
        Node current = this;
        while (!current.isRootNode()) {
            path.add(0, current);
            current = current.getParent();
        }
        // ensure the root node is added
        path.add(0, current);
        return path;
    }

    public String toString() {
        return "[parent=" + parent +  ", state="
                + getState() + ", pathCost=" + pathCost + "]";
    }

}