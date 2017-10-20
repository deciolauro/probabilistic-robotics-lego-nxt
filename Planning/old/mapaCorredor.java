import lejos.geom.*;
import lejos.robotics.mapping.LineMap;

public class mapaCorredor{

  public static void main(String[] args){
    Line[] lines = {
      /* 1o Quadrante */	
      /* L-shape polygon */
      new Line(45,524+200,245,330+200),
      new Line(245,330+200,347,425+200),
      new Line(260,505+200,347,425+200),
      new Line(260,505+200,415,665+200),
      new Line(305,775+200,415,665+200),
      new Line(305,775+200,45,524+200),
      /* Open Pentagon */
      new Line(740,-135+200,515,0+200),
      new Line(610,247+200,515,0+200),
      new Line(610,247+200,880,284+200),
      new Line(940,0+200,880,284+200),
      /* Triangle */
      new Line(850,545+200,1130,510+200),
      new Line(1009,850+200,1130,510+200),
      new Line(1009,850+200,850,545+200),

	  /* 2o Quadrante */	
      /* Open Pentagon */
      new Line(1535,573+200,1715,375+200),
      new Line(1935,550+200,1715,375+200),
      new Line(1935,550+200,1900,795+200),
      new Line(1630,780+200,1900,795+200),
      /* Triangle */
      new Line(1390,-101+200,1550,190+200),
      new Line(1210,263+200,1550,190+200),
      new Line(1210,263+200,1390,-101+200),
 
 	  /* 3o Quadrante */	
      /* L-shape polygon */
      new Line(1940,20+200,2315,130+200),
      new Line(2215,410+200,2315,130+200),
      new Line(2215,410+200,2095,360+200),
      new Line(2130,260+200,2095,360+200),
      new Line(2130,260+200,1885,175+200),
      new Line(1940,20+200,1885,175+200),
      /* Open L-shape polygon */
      new Line(2200,830+200,2255,685+200),
      new Line(2370,730+200,2255,685+200),
      new Line(2370,730+200,2443,510+200),   
      new Line(2600,560+200,2443,510+200), 
      new Line(2600,560+200,2455,908+200),
      /* Open Pentagon */
      new Line(2960,260+200,2770,433+200),
      new Line(2580,253+200,2770,433+200),
      new Line(2580,253+200,2635,0+200),
      new Line(2895,0+200,2635,0+200)
    };
    //Rectangle(int x, int y, int width, int height)  -- always integer coordinates
    //Creates a rectangle with top left corner at (x,y) and with specified width and height.
    //Rectangle bounds = new Rectangle(0, -841, 1189, 841);  
    Rectangle bounds = new Rectangle(0, 1030, 3020, 1200); 
    LineMap mymap = new LineMap(lines, bounds);

    try{
        mymap.createSVGFile("mapa.svg");
        mymap.flip().createSVGFile("mapaFlipY.svg"); //creates a fliped version in the Y-axis of the orginal image
    }catch (Exception e){
        System.out.print("Exception caught: ");
        System.out.println(e.getMessage());
    }
  }
}
