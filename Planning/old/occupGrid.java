/*
Projeto 5 - Parte C - Mapa de Ocupação
Dividir para conquistar:
- método para busca em largura (parâmetros: init, final, grafo   saída: plano)
- fazer grid do mapa de linhas (discretização)
- método da busca em largura, frente de onda 
*talvez colocar os bounds do grid como atributos da classe occupGrid*
*/
//import lejos.geom.*;
//import lejos.robotics.mapping.LineMap;
import java.util.Arrays;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.List;
import java.awt.Point;
import java.util.LinkedList;
import java.util.Queue;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Scanner;

public class occupGrid{
  
    public static int sizeX;
    public static int sizeY;
    public int width;
    public int height;
    public int cellSize;
    public int[][] map;
    public static ArrayList<Point> actions; 

     public occupGrid(int w, int h, int cs) {
        this.width = w;
        this.height = h;
        this.cellSize = cs;
        this.sizeX = (int)Math.ceil((float)w/cs);
        this.sizeY = (int)Math.ceil((float)h/cs);
        this.map = new int[this.sizeX][this.sizeY];
    }

    private void setActions(int type) {
        // 4-Neighborhood
        this.actions = new ArrayList<Point>();
        this.actions.add(new Point(-1,0)); // North
        this.actions.add(new Point(1,0)); // South
        this.actions.add(new Point(0,1)); // East
        this.actions.add(new Point(0,-1)); // West

        // 8-Neighborhood
        if(type == 8)
        {
            this.actions.add(new Point(-1,-1)); // NW
            this.actions.add(new Point(-1,1)); // NE
            this.actions.add(new Point(1,-1)); // SW
            this.actions.add(new Point(1,1)); // SE
        }
    }

    private void buildGrid() { 
        for (int row = 0; row < this.sizeX; row++ )
            for (int column = 0; column < this.sizeY; column++)
            {
                this.map[row][column] = 999; //initialize with 999
            }
    }

    private void printGrid() {
        for (int row = 0; row < this.sizeX; row++ ){
            for (int column = 0; column < this.sizeY; column++)
            {
                System.out.print(" "+ this.map[row][column]);
            }
            System.out.println("");
        }
    }

    private Set<Point> reachableCells(Point p) { // 4- neighboor  
        Point cell = new Point(p);
        Set<Point> neighboors = new HashSet<Point>();
        for (int i = 0; i < actions.size(); i++) {
            if ((((cell.x + actions.get(i).x) >= 0) && ((cell.y + actions.get(i).y) >=0)) && ((cell.x + actions.get(i).x) < this.sizeX) && ((cell.y + actions.get(i).y) < this.sizeY)){
                if (this.map[(cell.x + actions.get(i).x)][(cell.y + actions.get(i).y)] == -1)
                    continue; 
                cell.translate(actions.get(i).x, actions.get(i).y);
                neighboors.add(cell);
                //System.out.println("Actions: "+ cell); 
                cell = new Point(p);
            }
        }
        return neighboors;
        //System.out.println("Actions: "+ Arrays.toString(actions.toArray())); print actions
    }

    private List<Node> bfs(int gx, int gy, int sx, int sy){ //gx, gy - goal coordinates --- sx, sy - start coordinates
        Set<Point> explored = new HashSet<Point>(); //creates explored set of points
        Set<Point> inQueue = new HashSet<Point>(); //create inQueue set of points that are in the queue but were not explored
        Set<Point> nextCell;
        Queue<Node> fringe = new LinkedList<Node>();
        Point goal = new Point(gx, gy);
        Point init = new Point(sx,sy);
        Node nodeParent = new Node(goal); //wavefront starting from goal
        Node nodeSon;
        boolean foundInit = false;
        fringe.add(nodeParent);
        inQueue.add(goal);
        alterGrid(goal, 0);
        while (!foundInit && !fringe.isEmpty()){
            nodeParent = fringe.poll();
            inQueue.remove(nodeParent.getState());
            explored.add(nodeParent.getState());
            alterGrid(nodeParent.getState(),nodeParent.getPathCost());
            if (init.equals(nodeParent.getState())){
                foundInit = true;
                break;
            }
            nextCell = reachableCells(nodeParent.getState());
            for (Point p : nextCell) {
                if (!explored.contains(p) && !inQueue.contains(p)){
                    nodeSon = new Node(p,nodeParent,1);
                    fringe.add(nodeSon);
                    inQueue.add(p);
                }
            }
            nextCell.clear();
        }
        return nodeParent.getPathFromRoot();

    }

    /*
    Any operations that alterGrid performs on mapRef are such that, for all practical purposes, they are performed on map itself 
    (except when mapRef is changed to point to a different int[][] instance like mapRef = new int [sizeX][sizeY]) 
    */
    private void alterGrid(int i, int j, int val){
        this.map[i][j] = val;
    }
    private void alterGrid(Point p, int val){
        this.map[p.x][p.y] = val;
    }


    public static void main(String[] args){
        List<Node> nodePath;
        occupGrid oc = new occupGrid(10,16,2);
        oc.buildGrid();
        oc.setActions(4);
        Point obs = new Point(); 
        //setting obstacles
        oc.alterGrid(1,2,-1);
        oc.alterGrid(2,2,-1);
        oc.alterGrid(3,2,-1);
        oc.alterGrid(3,3,-1);
        //exemplo de como usar a classe Point
        obs.setLocation(2,5); 
        oc.alterGrid(obs.getLocation(),-1);
        //
        oc.alterGrid(1,5,-1);
        oc.alterGrid(1,6,-1);
        oc.alterGrid(2,6,-1);
        //
        nodePath = oc.bfs(4,7,2,3);
        oc.printGrid();
        Collections.reverse(nodePath);
        for (int i = 0; i < nodePath.size(); i++) 
            System.out.println(""+nodePath.get(i).getState());
        // Line[] lines = {
        //   /* L-shape polygon */
        //   new Line(170,437,60,680),
        //   new Line(60,680,398,800),
        //   new Line(398,800,450,677),
        //   new Line(450,677,235,595),
        //   new Line(235,595,281,472),
        //   new Line(281,472,170,437),
        //   /* Triangle */
        //   new Line(1070,815,770,602),
        //   new Line(770,602,1060,516),
        //   new Line(1070,815,1060,516),
        //   /* Pentagon */
        //   new Line(335,345,502,155),
        //   new Line(502,155,700,225),
        //   new Line(700,225, 725,490),
        //   new Line(725,490,480,525),
        //   new Line(480,525,335,345)

        // };
        // Rectangle bounds = new Rectangle(0, 0, 1195, 920); 
        // LineMap mymap = new LineMap(lines, bounds);
        // mapToGrid(1195,920,5); //testar com (20,20,5) -- esperado uma discretizacao de 4x4 celulas
        // try{
        //     mymap.createSVGFile("mapa.svg");
        //     mymap.flip().createSVGFile("mapaFlipY.svg"); //creates a fliped version in the Y-axis of the orginal image
        // }catch (Exception e){
        //     System.out.print("Exception caught: ");
        //     System.out.println(e.getMessage());
        // }
    }
}
