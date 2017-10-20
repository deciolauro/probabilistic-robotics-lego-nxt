import java.util.*;

public class ProbabilisticOccupancy extends SearchAlgorithm  {
    private static class Point {
        double distance;
        double cust;
        double collision;
        Point origin;
        int x;
        int y;
        Point (int x, int y) {
            this.x = x;
            this.y = y;
            origin = null;
            distance = -1;
            cust = 0;
            collision = 0;
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
    double alpha;
    int convolutions;

    double[][] gaussian  = {
        {0.05, 0.1, 0.05},
        {0.1, 0.4, 0.1},
        {0.05, 0.1, 0.05}
    };

    ProbabilisticOccupancy (Grid grid, double alpha, int convolutions) {
       super(grid);

        end = new Point(0, 0);
        init = new Point(grid.x-1, grid.y-1);

        data = new Point[grid.x][grid.y];

        queue = new PriorityQueue<Point>(1000, new Comparator<Point>() {
            public int compare(Point a, Point b) {
                return Double.compare(a.cust, b.cust);  
            }      
        });
        this.alpha = alpha;
        this.convolutions = convolutions;
    }

    private void conv () {
        double[][] tmp =  new double[g.x][g.y];
        for(int i = 0; i < g.x; i++) {
            for(int j = 0; j < g.y; j++) {

                for(int ii = 0; ii < gaussian.length; ii++) {
                    for(int jj = 0; jj < gaussian[ii].length; jj++) {
                        int x = i - gaussian.length/2 + ii;
                        int y = j - gaussian[ii].length/2 + jj;
                        // O robo colide qunado esta fora do mapa
                        double value = 1;
                        if (x < g.x && x >= 0 &&
                            y < g.y && y >= 0) {
                            value = g.get(x, y);
                        }
                        tmp[i][j] += value*gaussian[ii][jj];
                        tmp[i][j] = Math.min(1.0, tmp[i][j]);
                    }
                }
            }
        } 
        for(int i = 0; i < g.x; i++)
            for(int j = 0; j < g.y; j++)
                if (g.get(i, j) != 1)
                    g.set(i, j, tmp[i][j]);
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

        for (i = 1; i <= convolutions; i++)
            conv();

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

    // private void addQueue (int i, int j, Point origin) {
    //     if (i >= data.length || i < 0) return;
    //     if (j >= data[0].length || j < 0) return;
    //     if (g.get(i, j) == 1.0) return;
    //     if (data[i][j] == null) {
    //         Point p = new Point(i, j);
    //         p.collision = 1 - g.get(i, j);
    //         p.origin = origin;
    //         p.distance = p.distance(end);
    //         if (origin != null) {
    //             p.collision *= origin.collision;
    //             p.cust = origin.cust + origin.distance(p);
    //         }
    //         p.priority = p.cust + p.distance;
    //         data[i][j] = p;
    //         queue.add(p);
    //     }
    // }
    private void addQueue (int i, int j, Point origin) {
        if (i >= data.length || i < 0) return;
        if (j >= data[0].length || j < 0) return;
        if (g.get(i, j) == 1) return;
        if (data[i][j] == null) {
            Point p = new Point(i, j);
            p.collision = 1 - g.get(i, j);
            p.distance = p.distance(end);
            p.origin = origin;
            if (origin != null) {
                p.collision *= origin.collision;
                p.cust = alpha * origin.cust + origin.distance(p) + p.distance;
                p.cust += (1-alpha)*(p.collision);
            }
            data[i][j] = p;
            queue.add(p);
        }
    }
}
