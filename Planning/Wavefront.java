import java.util.*;

public class Wavefront extends SearchAlgorithm  {
    private static class Point {
        Point origin;
        int x;
        int y;
        Point (int x, int y) {
            this.x = x;
            this.y = y;
            origin = null;
        }
    }

    Point init;
    Point end;
    Point[][] data;
    Queue<Point> queue;

    Wavefront (Grid grid) {
       super(grid);

        end = new Point(0, 0);
        init = new Point(grid.x-1, grid.y-1);

        data = new Point[grid.x][grid.y];

        queue = new LinkedList<Point>();
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
            addQueue(i+1, j, p);
            addQueue(i-1, j, p);
            addQueue(i, j+1, p);
            addQueue(i, j-1, p);
        }

        p = data[end.x][end.y];
        if (p != null) {
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
            p.origin = origin;
            data[i][j] = p;
            queue.add(p);
        }
    }
}
