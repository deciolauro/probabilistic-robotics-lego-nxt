public class Test  {

    public static void makeMap (Grid g) {
        g.addLine(170,437,60,680);
        g.addLine(60,680,398,800);
        g.addLine(398,800,450,677);
        g.addLine(450,677,235,595);
        g.addLine(235,595,281,472);
        g.addLine(281,472,170,437);
        /* Triangle */
        g.addLine(1070,815,770,602);
        g.addLine(770,602,1060,516);
        g.addLine(1070,815,1060,516);
        /* Pentagon */
        g.addLine(335,345,502,155);
        g.addLine(502,155,700,225);
        g.addLine(700,225, 725,490);
        g.addLine(725,490,480,525);
        g.addLine(480,525,335,345);

        g.addLine(100,200,500,1000);

    }

    public static void main(String[] args) {
        /* AStar */
        int size = 5; // discretizacao em size x size;
        Grid grid = new Grid(size, 1200, 900);
        SearchAlgorithm salg = new AStar(grid);
        makeMap(grid);

        salg.setInit (93.0, 737.0);
        salg.setEnd (994.0, 188.0);
        int cust = salg.search ();
        salg.save ("images/astar");
        System.out.println("astar: "+cust);


        /* BestChoice */
        grid = new Grid(size, 1200, 900);
        salg = new BestChoice(grid);
        makeMap(grid);

        salg.setInit (93.0, 737.0);
        salg.setEnd (994.0, 188.0);
        cust = salg.search ();
        salg.save ("images/best");
        System.out.println("best: "+cust);

        /* Wavefront */
        grid = new Grid(size, 1200, 900);
        salg = new Wavefront(grid);
        makeMap(grid);

        salg.setInit (93.0, 737.0);
        salg.setEnd (994.0, 188.0);
        cust = salg.search ();
        salg.save ("images/wavefront");
        System.out.println("wavefront: "+cust);


        /* ProbabilisticOccupancy */
        grid = new Grid(size, 1200, 900);
        salg = new ProbabilisticOccupancy(grid, 0.8, 300);
        makeMap(grid);

        salg.setInit (93.0, 737.0);
        salg.setEnd (994.0, 188.0);
        cust = salg.search ();
        salg.save ("images/probabilistic");
        System.out.println("probabilistic: "+cust);

        /* Dilation */
        Grid.main(null);

    }
}
