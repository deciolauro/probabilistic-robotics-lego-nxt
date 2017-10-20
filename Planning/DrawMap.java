import java.awt.BorderLayout;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;

import javax.swing.JFrame;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Point;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionListener;
import java.awt.event.MouseWheelEvent;
import java.awt.event.MouseWheelListener;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;

import javax.imageio.ImageIO;
import javax.swing.JOptionPane;
import javax.swing.JPanel;

class ImageMap extends JPanel {
    private class Point3D {
        public double x, y, z;
        public Point3D(double x, double y, double z) {
            this.x = x;
            this.y = y;
            this.z = z; // em radianos
        }
    }

    private double MAX_ZOOM = 1.0;      // pixel per cm
    private double MIN_ZOOM = 20.0;     // pixel per cm
    private double GRID_SIZE = 10.0;    // cm

    private double zoom = 2.0;
    private double centerx = 0.0;
    private double centery = 0.0; // cm
    private Point mousePt;

    private ArrayList<Point3D> points;
    private int poseViewMode = 0;
    private boolean poseLine = false;
    
    public ImageMap() {
        points = new ArrayList<Point3D>();
        setBackground(Color.BLACK);
        addMouseWheelListener(new MouseWheelListener() {
            @Override
            public void mouseWheelMoved(MouseWheelEvent e) {
                // TODO Auto-generated method stub
                if(e.getWheelRotation()<0){
                    if (zoom < MIN_ZOOM)
                        zoom *= 1.1;
                    repaint();
                }
                //Zoom out
                if(e.getWheelRotation()>0){
                    if (zoom > MAX_ZOOM)
                        zoom /= 1.1;
                    repaint();
                }
            }
        });
        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                mousePt = e.getPoint();
                repaint();
            }
        });

        addMouseMotionListener(new MouseMotionListener() {
            @Override
            public void mouseMoved(MouseEvent e) {
                // TODO Auto-generated method stub
            }
            
            @Override
            public void mouseDragged(MouseEvent e) {
                centerx += e.getX() - mousePt.x;
                centery += e.getY() - mousePt.y;
                mousePt = e.getPoint();
                repaint();
            }
        });
    }
    private void drawModel (Graphics g) {
        int width = (int) (getWidth()+2*centerx);
        int height = (int) (getHeight()+2*centery);
        int count = 0;
        int x_tmp = 0, y_tmp = 0;
        
        for (Point3D p : points) {
            int x = width/2+(int)(p.x*zoom);
            int y = height/2+(int)(p.y*zoom)*-1;
            
            if (poseViewMode == 0) {
                g.setColor(Color.getHSBColor((float) (p.z/(2.0*Math.PI)), 1, 1));
                g.fillOval(
                        x-(int)(zoom/2.0*1.5),
                        y-(int)(zoom/2.0*1.5),
                        (int)(zoom*1.5),
                        (int)(zoom*1.5)
                );
            } else if (poseViewMode == 1) {
                g.setColor(Color.RED);
                g.drawLine(
                    width/2+(int)(p.x*zoom),
                    height/2-(int)(p.y*zoom), 
                    width/2+(int)(p.x*zoom+Math.sin(p.z)*zoom),
                    height/2-(int)(p.y*zoom-Math.cos(p.z)*zoom)
                );

               g.drawLine(
                    width/2+(int)(p.x*zoom+zoom*Math.sin(p.z)),
                    height/2-(int)(p.y*zoom-zoom*Math.cos(p.z)),
                    width/2+(int)(p.x*zoom+0.6*zoom*Math.sin(Math.PI/8+p.z)),
                    height/2-(int)(p.y*zoom-0.6*zoom*Math.cos(Math.PI/8+p.z))
                );
            } else if (poseViewMode == 2) {
                g.setColor(Color.RED);
                g.fillOval(
                        x-(int)(zoom/2.0*1.5),
                        y-(int)(zoom/2.0*1.5),
                        (int)(zoom*1.5),
                        (int)(zoom*1.5)
                );
                g.setColor(Color.BLACK);
                g.drawLine(
                    width/2+(int)(p.x*zoom),
                    height/2-(int)(p.y*zoom), 
                    width/2+(int)(p.x*zoom+Math.sin(p.z)*zoom),
                    height/2-(int)(p.y*zoom-Math.cos(p.z)*zoom)
                );
            }

            if (poseLine && count != 0) {
                g.setColor(Color.LIGHT_GRAY);
                g.drawLine(x_tmp, y_tmp, x, y);
            }

            x_tmp = x;
            y_tmp = y;
            count++;
        }
    }
    
    @Override
    protected void paintComponent(Graphics g) {
        int width = (int) (getWidth());
        int height = (int) (getHeight());
        int width2 = (int) (getWidth()+2*centerx);
        int height2 = (int) (getHeight()+2*centery);
        super.paintComponent(g);
    
        g.setColor(new Color(20, 20, 20));
        
        int initial_x = height2/2;
        while (initial_x < width) {
            initial_x += GRID_SIZE*zoom;
            g.drawLine(0, initial_x, width, initial_x); 
        }
        initial_x = height2/2;
        while (initial_x > 0) {
            initial_x -= GRID_SIZE*zoom;
            g.drawLine(0, initial_x, width, initial_x); 
        }
        int initial_y = width2/2;
        while (initial_y < width) {
            initial_y += GRID_SIZE*zoom;
            g.drawLine(initial_y, 0, initial_y, height);
        }
        initial_y = width2/2;
        while (initial_y > 0) {
            initial_y -= GRID_SIZE*zoom;
            g.drawLine(initial_y, 0, initial_y, height);
        }

        g.setColor(Color.ORANGE);
        g.drawLine(width2/2, 0, width2/2, height);
        g.drawLine(0, height2/2, width, height2/2);
        
        drawModel(g);
    }
    
    /**
     * Adiciona um ponto ao mapa
     * @param p ponto
     */
    public void addPoint(double x, double y, double t) {
        Point3D point = new Point3D(x, y, t);
        points.add(point);
        repaint();
    }
    
    public void showLine () {
        poseLine = !poseLine;
        repaint();
    }
    
    public void setVisual (int method) {
        poseViewMode = method;
        repaint();
    }

    public void square (int r, int g, int b) {

    }

    public void save () {
        Integer name = new Integer((int) (Math.random()*10000000));
        BufferedImage imagebuf = new BufferedImage(getWidth(), getHeight(), BufferedImage.TYPE_INT_RGB);
        Graphics g = imagebuf.createGraphics();
        g.fillRect(0, 0, imagebuf.getWidth(), imagebuf.getHeight());
        print(g);
        try {
            ImageIO.write(imagebuf, "png",  new File(name.toString()+".png"));
            JOptionPane.showMessageDialog(null, "Image saved.");
        } catch (IOException e) {
            e.printStackTrace();
            JOptionPane.showMessageDialog(null, "Image not saved.");
        }
    }
}

public class DrawMap extends JPanel {
    private ImageMap map;
    
    public DrawMap () {
        JFrame frame = new JFrame("Mapa MAC0318");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());
        this.map = new ImageMap();
        frame.add(this.map);
        frame.setSize(500, 500);
        frame.setVisible(true);
        
        frame.setFocusable(true);
        frame.requestFocusInWindow();
        
        frame.addKeyListener(new KeyListener() {
            @Override
            public void keyTyped(KeyEvent e) {
            }
            @Override
            public void keyReleased(KeyEvent e) {
            }
            @Override
            public void keyPressed(KeyEvent e) {
                char input = e.getKeyChar();
                if (input == '1') map.setVisual(0);
                else if (input == '2') map.setVisual(1);
                else if (input == '3') map.setVisual(2);
                else if (input == 'l') map.showLine();
                else if (input == 's') map.save();
            }
        });
    }

    public DrawMap (boolean showHelp) {
        this();
        String text = "1,2,3 - Change view mode.\n";
        text += "s - Save image.\n";
        text += "l - Show trace.\n";
        JOptionPane.showMessageDialog(null, text);
    }

    public static void main(String[] args) {
        DrawMap map = new DrawMap(true);
    }
}
