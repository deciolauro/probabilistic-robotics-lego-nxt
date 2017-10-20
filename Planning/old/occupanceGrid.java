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
//import java.util.PriorityQueue; 
import java.util.LinkedList;
import java.util.Queue;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Scanner;

public class occupanceGrid{

    public int width;
    public int height;
    public int gridX;
    public int gridY;
    public int cellSize;
    public int[][] map;
    public ArrayList<Point> actions;

    public occupanceGrid(int w, int h, int cs) {
        this.width = w;
        this.height = h;
        this.cellSize = cs;
        this.gridX = (int)Math.ceil((float)w/cs);
        this.gridY = (int)Math.ceil((float)h/cs);
        this.map = new int[this.gridX][this.gridY];
    }

    public void setActions(int type) {
        // 4-Neighborhood
        this.actions = new ArrayList<Point>();
        this.actions.add(new Point(-1,0)); // North
        this.actions.add(new Point(1,0)); // South
        this.actions.add(new Point(0,1)); // East
        this.actions.add(new Point(0,-1)); // West

        // 8-Neighborhood
        if(type == 8) {
            this.actions.add(new Point(-1,-1));
            this.actions.add(new Point(-1,1));
            this.actions.add(new Point(1,-1));
            this.actions.add(new Point(1,1));
        }
    }


    public void buildGrid() {
        for(int row=0; row<this.gridX; row++) {
            for(int column=0; column<this.gridY; column++) {
                this.map[row][column] = 999; //initialize with 999
            }
        }
    }


    public void printGrid() {
        for(int row=0; row<this.gridX; row++) {
            for(int column=0; column<this.gridY; column++) {
                if(this.map[row][column] < 0) {
                    System.out.print("  " + this.map[row][column]);
                }
                else if(this.map[row][column] < 10) {
                    System.out.print("   " + this.map[row][column]);
                }
                else if(this.map[row][column] < 100) {
                    System.out.print("  " + this.map[row][column]);
                }
                else {
                    System.out.print(" " + this.map[row][column]);
                }
            }
            System.out.println("");
        }
    }


    //public static Set<Point> reachableCells(int[][] map, Point p)
    //{
    //  Point cell = new Point(p);
    //  Set<Point> neighboors = new HashSet<Point>();
    //  for (int i = 0; i < actions.size(); i++) {
    //      if ((((cell.x + actions.get(i).x) >= 0) && ((cell.y + actions.get(i).y) >=0)) && ((cell.x + actions.get(i).x) < sizeX) && ((cell.y + actions.get(i).y) < sizeY)) {
    //          if (map[(cell.x + actions.get(i).x)][(cell.y + actions.get(i).y)] == -1)
    //              continue; 
    //          cell.translate(actions.get(i).x, actions.get(i).y);
    //          neighboors.add(cell);
    //          //System.out.println("Actions: "+ cell); 
    //          cell = new Point(p);
    //      }
    //  }
    //  return neighboors;
    //  //System.out.println("Actions: "+ Arrays.toString(actions.toArray())); print actions
    //}


    //public static List<Node> bfs(int[][] grid, int gx, int gy, int sx, int sy)
    //{ //gx, gy - goal coordinates --- sx, sy - start coordinates
    //  Set<Point> explored = new HashSet<Point>(); //creates explored set of points
    //  Set<Point> inQueue = new HashSet<Point>(); //create inQueue set of points that are in the queue but were not explored
    //  Set<Point> nextCell;
    //  //PriorityQueue<Node> fringe = new PriorityQueue<Node>(20); // creates queue of nodes
    //  Queue<Node> fringe = new LinkedList<Node>();
    //  Point goal = new Point(gx, gy);
    //  Point init = new Point(sx,sy);
    //  Node nodeParent = new Node(goal); //wavefront starting from goal
    //  Node nodeSon;
    //  boolean foundInit = false;
    //  fringe.add(nodeParent);
    //  inQueue.add(goal);
    //  alterGrid(grid, goal, 0);
    //  while (!foundInit && !fringe.isEmpty()) {
    //      nodeParent = fringe.poll();
    //      inQueue.remove(nodeParent.getState());
    //      explored.add(nodeParent.getState());
    //      alterGrid(grid,nodeParent.getState(),nodeParent.getPathCost());
    //      if (init.equals(nodeParent.getState())) {
    //          foundInit = true;
    //          break;
    //      }
    //      nextCell = reachableCells(grid,nodeParent.getState());
    //      for (Point p : nextCell) {
    //          if (!explored.contains(p) && !inQueue.contains(p)) {
    //              nodeSon = new Node(p,nodeParent,1);
    //              fringe.add(nodeSon);
    //              inQueue.add(p);
    //          }
    //      }
    //      nextCell.clear();
    //  }
    //  return nodeParent.getPathFromRoot();
    //}

 /*
    Any operations that alterGrid performs on mapRef are such that, for all practical purposes, they are performed on map itself 
    (except when mapRef is changed to point to a different int[][] instance like mapRef = new int [sizeX][sizeY]) 
 */
    public void alterGrid(int i, int j, int val) {
        this.map[i][j] = val;
    }


    public void alterGrid(Point p, int val) {
        this.map[p.x][p.y] = val;
    }


    public void buildVertical(Point p0, Point p1) {
        Point pLow, pHigh;

        if(p0.y > p1.y) {
            pLow = p1;
            pHigh = p0;
        }
        else {
            pLow = p0;
            pHigh = p1;
        }

        while(pLow.y <= pHigh.y) {
            //System.out.println(pHigh);
            //System.out.println(pLow);
            this.alterGrid(pLow, -1);
            pLow.y += 1;
        }
    }


    public void buildLine(Point p0, Point p1) {
        // Deal with vertical lines
        if (((int)Math.ceil((float)p0.x/this.cellSize)) == ((int)Math.ceil((float)p1.x/this.cellSize))) {
            Point p0fix = new Point((int)Math.ceil((float)p0.x/this.cellSize), this.gridY - (int)Math.ceil((float)p0.x/this.cellSize));
            Point p1fix = new Point((int)Math.ceil((float)p1.x/this.cellSize), this.gridY - (int)Math.ceil((float)p1.x/this.cellSize));
            //System.out.println(p0);
            //System.out.println(p0fix);
            //System.out.println(p1);
            //System.out.println(p1fix);
            this.buildVertical(p0fix, p1fix);
        }
    }


  public static void main(String[] args){
    Scanner stop = new Scanner(System.in);
    Scanner token;
    occupanceGrid oc = new occupanceGrid(10,16,2);
    oc.buildGrid();
    oc.setActions(4);
    //List<Node> nodePath;


    Point test1 = new Point(3, 2);
    Point test2 = new Point(4, 11);
    oc.buildLine(test1, test2);
    oc.printGrid();
    oc.buildLine(test2, test1);
    oc.printGrid();

    //token = new Scanner(stop.readLine());
    

    //Point obs = new Point(); 
    ////setting obstacles
    //alterGrid(map,1,2,-1);
    //alterGrid(map,2,2,-1);
    //alterGrid(map,3,2,-1);
    //alterGrid(map,3,3,-1);
    ////exemplo de como usar a classe Point
    //obs.setLocation(2,5); 
    //alterGrid(map,obs.getLocation(),-1);
    ////
    //alterGrid(map,1,5,-1);
    //alterGrid(map,1,6,-1);
    //alterGrid(map,2,6,-1);
    ////
    //nodePath = bfs(map,4,7,2,3);
    //printGrid(map,10,16,2);
    //Collections.reverse(nodePath);
    //for (int i = 0; i < nodePath.size(); i++) 
    //  System.out.println(""+nodePath.get(i).getState());
    //// Line[] lines = {
    ////   /* L-shape polygon */
    ////   new Line(170,437,60,680),
    ////   new Line(60,680,398,800),
    ////   new Line(398,800,450,677),
    ////   new Line(450,677,235,595),
    ////   new Line(235,595,281,472),
    ////   new Line(281,472,170,437),
    ////   /* Triangle */
    ////   new Line(1070,815,770,602),
    ////   new Line(770,602,1060,516),
    ////   new Line(1070,815,1060,516),
    ////   /* Pentagon */
    ////   new Line(335,345,502,155),
    ////   new Line(502,155,700,225),
    ////   new Line(700,225, 725,490),
    ////   new Line(725,490,480,525),
    ////   new Line(480,525,335,345)

    //// };
    //// Rectangle bounds = new Rectangle(0, 0, 1195, 920); 
    //// LineMap mymap = new LineMap(lines, bounds);
    //// mapToGrid(1195,920,5); //testar com (20,20,5) -- esperado uma discretizacao de 4x4 celulas
    //// try{
    ////     mymap.createSVGFile("mapa.svg");
    ////     mymap.flip().createSVGFile("mapaFlipY.svg"); //creates a fliped version in the Y-axis of the orginal image
    //// }catch (Exception e){
    ////     System.out.print("Exception caught: ");
    ////     System.out.println(e.getMessage());
    //// }
  }
}
