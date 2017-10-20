/*
Projeto 5 - Parte C - Mapa de Ocupação
Dado um lineMap transformar em um grid
*/
        
import java.awt.image.BufferedImage;
import javax.imageio.ImageIO;
import java.io.IOException;
import java.io.File;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Image;
import java.awt.Color;


public class Grid implements Cloneable {

    private double[][] matriz;
    private boolean[][] red;
    private double size;
    public int x, y;

    /**
     * @brief Classe para desenhar segmentos em uma matriz
     * @details Vai criar na memoria um mapa de tamanho x por y onde
     * 
     * 
     * @param squareSize tamanho dos pontos
     * @param x comprimento vertical
     * @param y comprimento horizontal
     */
    public Grid (double squareSize, double x, double y) {
        size = squareSize;
        matriz = new double[getI(x)][getJ(y)];
        red = new boolean[getI(x)][getJ(y)];
        this.x = getI(x);
        this.y = getJ(y);
    }

    public void addLine (double ax, double ay, double bx, double by) {
        ax /= size;
        bx /= size;
        ay /= size;
        by /= size;
        double dx = bx - ax;
        double dy = by - ay;
        double tmp;
        if (Math.abs(dx) > Math.abs(dy)) {
            if (bx > ax) {
                tmp = bx;
                bx = ax;
                ax = tmp;
                tmp = by;
                by = ay;
                ay = tmp;
            }
            for (int cx = (int) (bx+0.5); cx < ax; cx++) {
                int cy = (int)(ay + dy * (cx - ax) / dx);
                if (cx >= 0 && cx < x && cy >= 0 && cy < y)
                    matriz[cx][cy] = 1;
            }
        } else {
            if (by > ay) {
                tmp = bx;
                bx = ax;
                ax = tmp;
                tmp = by;
                by = ay;
                ay = tmp;
            }
            for (int cy = (int) (by+0.5); cy < ay; cy++) {
                int cx = (int)(ax + dx * (cy - ay) / dy);
                if (cy >= 0 && cy < y && cx >= 0 && cx < x)
                    matriz[cx][cy] = 1;
            }
        }
    }

    public String toString() {
        String data = "";
        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {
                data += matriz[i][j] > 0 ? 1: 0;
            }
            data += '\n';
        }
        return data;
    }

    public void set(double x, double y, double value) {
        matriz[getI(x)][getJ(y)] = value;
    }

     public void set(int x, int y, double value) {
        matriz[x][y] = value;
    }

    public double get(double x, double y) {
        return matriz[getI(x)][getJ(y)];
    }

    public double get(int x, int y) {
        return matriz[x][y];
    }

    public int getI(double x) {
        x /= size;
        return (int) x;
    }


    public int getJ(double y) {
        y /= size;
        return (int) y;
    }

    public void mark (double x, double y) {
        red[getI(x)][getJ(y)] = true;
    }

    public void mark (int x, int y) {
        red[x][y] = true;
    }

    public void savePNG (String name, int sacale) throws IOException {
        int type = BufferedImage.TYPE_INT_ARGB;
        BufferedImage im = new BufferedImage(x, y, type);

        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {
                Color color;
                if (red[i][y-j-1]) {
                    color = new Color(1.0f, 0f, 0f);
                } else {
                    float value = (float)(1-matriz[i][y-j-1]);
                    color = new Color(value, value, value);
                }
                im.setRGB(i, j, color.getRGB());
            }
        }

        Image scaledImage = im.getScaledInstance(x * sacale, y * sacale, Image.SCALE_SMOOTH);
        BufferedImage newImage = new BufferedImage(
            scaledImage.getWidth(null),
            scaledImage.getHeight(null),
            type
        ); 
        Graphics2D graph = newImage.createGraphics();
        graph.drawImage(scaledImage, 0, 0, null);
        graph.dispose(); 
        File outputfile = new File(name+".png");
        ImageIO.write(newImage, "png", outputfile);
    }

    public void dilation () {
        boolean[][] mask  = {
            {true,true,true},
            {true,true,true},
            {true,true,true}
        };
        dilation(mask);
    }

    public void dilation (boolean[][] v) {
        if (v.length%2 == 0 || v[0].length%2 == 0)
            return;
        boolean[][] copy = new boolean[x][y];
        
        for(int i = 0; i < x; i++) {
            for(int j = 0; j < y; j++) {
                if (get(i, j) >= 1.0)
                    copy[i][j] = true;
            }
        }

        for(int i = 0; i < x; i++) {
            for(int j = 0; j < y; j++) {
                if (!copy[i][j]) continue;
                for(int ii = 0; ii < v.length; ii++) {
                    for(int jj = 0; jj < v[ii].length; jj++) {
                        int xx = i - v.length/2 + ii;
                        int yy = j - v[ii].length/2 + jj;
                        if (xx < x && xx >= 0 &&
                            yy < y && yy >= 0 &&
                            v[ii][jj]) {
                            set(xx, yy, get(i, j));
                        }

                    }
                }
            }
        }
    }

    public static void main(String[] args) {
        Grid grid =  new Grid(5, 1200, 900);
        grid.addLine(170,437,60,680);
        grid.addLine(60,680,398,800);
        grid.addLine(398,800,450,677);
        grid.addLine(450,677,235,595);
        grid.addLine(235,595,281,472);
        grid.addLine(281,472,170,437);
        /* Triangle */
        grid.addLine(1070,815,770,602);
        grid.addLine(770,602,1060,516);
        grid.addLine(1070,815,1060,516);
        /* Pentagon */
        grid.addLine(335,345,502,155);
        grid.addLine(502,155,700,225);
        grid.addLine(700,225, 725,490);
        grid.addLine(725,490,480,525);
        grid.addLine(480,525,335,345);

        boolean[][] mask  = {
            {true,true,true},
            {true,true,true},
            {true,true,true}
        };
        grid.dilation(mask);
        grid.dilation(mask);
        grid.dilation(mask);
        grid.dilation(mask);

        grid.mark(40, 40);

        try {
            grid.savePNG ("images/dilation", 4);
        } catch (IOException e) {
            System.out.println("Error to save image.");
        }
    }
}
