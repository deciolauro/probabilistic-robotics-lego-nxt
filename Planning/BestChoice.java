import java.util.*;

public class BestChoice extends SearchAlgorithm  {
    private static class Point {
        double distance;
        Point origin;
        int x;
        int y;
        Point (int x, int y) {
            this.x = x;
            this.y = y;
            origin = null;
            distance = -1;
        }
        public double distance (Point a) {
            double distance = (x-a.x)*(x-a.x)+(y-a.y)*(y-a.y);
            return distance;
        }
    }

    Point init;
    Point end;
    Point[][] data;
    Queue<Point> queue;

    BestChoice (Grid grid) {
       super(grid);

        end = new Point(0, 0);
        init = new Point(grid.x-1, grid.y-1);

        data = new Point[grid.x][grid.y];

        queue = new PriorityQueue<Point>(1000, new Comparator<Point>() {
            public int compare(Point a, Point b) {
                return Double.compare(a.distance, b.distance);  
            }      
        });
    }

    public void setInit (Double x, Double y) {
        init = new Point(g.getI(x), g.getJ(y));
    }

    public void setEnd (Double x, Double y) {
        end = new Point(g.getI(x), g.getJ(y));
    }

    public int search () {
        addQueue(init.x, init.y, null);

        Point p;
        int i;
        int j;
        int count = -1;

        while ((p = queue.poll()) != null) {
            i = p.x;
            j = p.y;
            if (p.distance == 0) break;
            addQueue(i+1, j, p);
            addQueue(i-1, j, p);
            addQueue(i, j+1, p);
            addQueue(i, j-1, p);
        }

        if (p != null && end.x == p.x && end.y == p.y) {
            count = 0;
            while (p != null) {
                g.mark(p.x, p.y);
                p = p.origin;
                count++;
            }
        }
        return count;
    }

    private void addQueue (int i, int j, Point origin) {
        if (i >= data.length || i < 0) return;
        if (j >= data[0].length || j < 0) return;
        if (g.get(i, j) != 0) return;
        if (data[i][j] == null) {
            Point p = new Point(i, j);
            p.distance = p.distance(end);
            p.origin = origin;
            data[i][j] = p;
            queue.add(p);
        }
    }
}
