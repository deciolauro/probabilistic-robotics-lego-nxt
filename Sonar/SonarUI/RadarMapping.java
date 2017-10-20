import java.awt.geom.Point2D;
import java.util.ArrayList;

import lejos.geom.*;
import lejos.robotics.mapping.LineMap;

public class RadarMapping
{
	Point2D currentPos;
	ArrayList<Point2D.Float> auxConvertedForRecursion;
	ArrayList<Point2D> scanConvertedToCartesian;
	//Line[] toPlot;
	ArrayList<Line> toPlot;
	int THRESHOLD = 4;
	int DISCARD = 240;

	public RadarMapping()
	{
		this.currentPos = new Point2D.Float(0.0f, 0.0f);
		this.scanConvertedToCartesian = new ArrayList<Point2D>();
		this.auxConvertedForRecursion = new ArrayList<Point2D.Float>();
		this.toPlot = new ArrayList<Line>();
		//this.toPlot = new Line[100];
	}
	
	public RadarMapping(float x, float y)
	{
		this.currentPos = new Point2D.Float(x, y);
		this.scanConvertedToCartesian = new ArrayList<Point2D>();
		this.auxConvertedForRecursion = new ArrayList<Point2D.Float>();
		this.toPlot = new ArrayList<Line>();
		//this.toPlot = new Line[100];
	}

	public ArrayList<Line> getToPlot()
	{
		return this.toPlot;
	}	
	public Point2D getPosition()
	{
		return this.currentPos;
	}
	
	public void setPosition(float x, float y)
	{
		this.currentPos.setLocation(x, y);
	}
	
	public void setXPosition(float x)
	{
		this.currentPos.setLocation(x, this.currentPos.getY());
	}
	
	public void setYPosition(float y)
	{
		this.currentPos.setLocation(this.currentPos.getX(), y);
	}
	
	public float distPointLine(Point2D.Float point, Point2D.Float vertex1, Point2D.Float vertex2)
	{
		float pv1x, pv1y, v1v2x, v1v2y, dot, lenSq, param, epsilon;
	
		epsilon = (float)(Math.pow(10.0, -5.0));
		pv1x = (float)(point.getX() - vertex1.getX());
		pv1y = (float)(point.getY() - vertex1.getY());
		v1v2x = (float)(vertex2.getX() - vertex1.getX());
		v1v2y = (float)(vertex2.getY() - vertex1.getY());
		
		dot = (pv1x*v1v2x) + (pv1y*v1v2y);
		lenSq = (float)((Math.pow((double)v1v2x, 2.0)) + (Math.pow((double)(v1v2y), 2.0)));
		param = -1.0f;
		
		if(Math.abs((double)lenSq) < epsilon) // Line of length 0
		{
			param = dot / lenSq;
		}
		
		float xf, yf;
  
		if (param < 0) // Landed before vertex 1
		{
			xf = (float)(vertex1.getX());
			yf = (float)(vertex1.getY());
		}
		else if (param > 1) // Landed after vertex 2
		{
			xf = (float)(vertex2.getX());
			yf = (float)(vertex2.getY());
		}
		else // Landed between
		{
			xf = (float)(vertex1.getX() + param * v1v2x);
			yf = (float)(vertex1.getY() + param * v1v2y);
		}
		
		float dx, dy;

		dx = (float)point.getX() - xf;
		dy = (float)point.getY() - yf;
		
		return (float)Math.sqrt(dx * dx + dy * dy);
	}
	
	public Point2D polarToCartesian(int radius, int angleDegrees)
	{
		float x, y;
		Point2D converted;
		
		x = (float)radius * (float)Math.cos((angleDegrees*2*Math.PI)/360.0);
		y = (float)radius * (float)Math.sin((angleDegrees*2*Math.PI)/360.0);
		converted = new Point2D.Float(x, y);
		
		return converted;
	}
	
	public void convertFullScanFromPolar(ArrayList<Integer> fullScan)
	{
		for(int i=0; i<fullScan.size(); i++)
		{
			Point2D converted = polarToCartesian(fullScan.get(i), 2*i);
			this.scanConvertedToCartesian.add(converted);
		}
	}
	
	public void recursiveConvertByPoints(int startIndex, int endIndex)
	{
		float localThreshold, currentMaxDistance;
		int currentIndex;
		
		localThreshold = 4.0f;
		currentMaxDistance = -1.0f;
		currentIndex = startIndex;
		
		if(startIndex >= endIndex) // Just to be safe
		{
			return;
		}
		//else if(endIndex-startIndex == 1) // To avoid plotting points too close
		//{
		//	return;
		//}
		else
		{
			for(int i=startIndex; i<endIndex; i++)
			{
				float cDist = distPointLine(this.auxConvertedForRecursion.get(i), this.auxConvertedForRecursion.get(startIndex), this.auxConvertedForRecursion.get(endIndex));
				// System.out.println(cDist);
				if( cDist > currentMaxDistance)
				{
					currentMaxDistance = cDist;
					currentIndex = i;
				}
			}
			if(currentMaxDistance < localThreshold)
			{
				this.scanConvertedToCartesian.add(this.auxConvertedForRecursion.get(startIndex));
				this.scanConvertedToCartesian.add(this.auxConvertedForRecursion.get(endIndex));
				return;
			}
			else
			{
				recursiveConvertByPoints(startIndex, currentIndex);
				recursiveConvertByPoints(currentIndex, endIndex);
			}
		}
	}
	
	public void recursiveConvert(ArrayList<Integer> fullScan)
	{
		for(int i=0; i<fullScan.size(); i++)
		{
			Point2D converted = polarToCartesian(fullScan.get(i), 2*i);
			// Another dumb java fix
			Point2D.Float converted2 = new Point2D.Float((float)converted.getX(), (float)converted.getY());
			// System.out.println(converted2);			
			this.auxConvertedForRecursion.add(converted2);
		}
		recursiveConvertByPoints(0, this.auxConvertedForRecursion.size()-1);
	}

	public void smartConvertFullScanFromPolar(ArrayList<Integer> fullScan)
	{
		Point2D start;// = polarToCartesian(fullScan.get(0), 0);
		Point2D end;
		int currentRange = fullScan.get(0);
		int i = 0;
		
		while(fullScan.get(i) > DISCARD)
		{
			i += 1;
		}
		
		start = polarToCartesian(fullScan.get(i), 2*i);
		end = polarToCartesian(fullScan.get(i), 2*i);
		
		currentRange = fullScan.get(i);
		i += 1;
		
		for(; i<fullScan.size(); i++)
		{
			if(Math.abs(currentRange-fullScan.get(i)) < THRESHOLD)
			{
				end = polarToCartesian(fullScan.get(i), 2*i);
			}
			else
			{
				this.scanConvertedToCartesian.add(start);
				this.scanConvertedToCartesian.add(end);
				start = polarToCartesian(fullScan.get(i), 2*i);
				currentRange = fullScan.get(i);
			}
		}
	}
	
	public void createLinesNaive()
	{
		if(this.scanConvertedToCartesian.isEmpty())
		{
			System.out.println("Must perform a fullScan first");
			System.exit(1);
		}
		
		for(int i=0; i<this.scanConvertedToCartesian.size()-1; i++)
		{
			// Null problem
			if((this.currentPos.distance(this.scanConvertedToCartesian.get(i))>240) || (this.currentPos.distance(this.scanConvertedToCartesian.get(i+1))>240)) {}
			else if((this.currentPos.distance(this.scanConvertedToCartesian.get(i))<3) || (this.currentPos.distance(this.scanConvertedToCartesian.get(i+1))<3)) {}
			else
			{
				float xi, yi, xf, yf;
				xi = (float)this.scanConvertedToCartesian.get(i).getX();
				yi = (float)this.scanConvertedToCartesian.get(i).getY();
				xf = (float)this.scanConvertedToCartesian.get(i+1).getX();
				yf = (float)this.scanConvertedToCartesian.get(i+1).getY();
				this.toPlot.add(new Line(xi, yi, xf, yf));
				//this.toPlot[i] = new Line(xi, yi, xf, yf);
			}
		}
		
	}
	
	public void createArrayToDraw(ArrayList<Point2D> scan)
	{
		if(this.scanConvertedToCartesian.isEmpty())
		{
			System.out.println("Must perform a fullScan first");
			System.exit(1);
		}
		
		for(int i=0; i<scan.size(); i++)
		{
			float xi, yi, xf, yf;
			xi = (float)this.currentPos.getX() + (float)this.scanConvertedToCartesian.get(i).getX();
			yi = (float)this.currentPos.getY() + (float)this.scanConvertedToCartesian.get(i).getY();
			xf = (float)this.currentPos.getX() + (float)this.scanConvertedToCartesian.get(i+1).getX();
			yf = (float)this.currentPos.getY() + (float)this.scanConvertedToCartesian.get(i+1).getY();
			this.toPlot.add(new Line(xi, yi, xf, yf));
			//this.toPlot[i] = new Line(xi, yi, xf, yf);
		}

	}

	public void inspectToPlot()
	{
		for(Line l : this.toPlot)
		{
			System.out.println(l);
		}
	}

	public void plotMap(String filename)
	{
		Rectangle bounds = new Rectangle(0, 0, 1189, 841);
		Line[] dumbJavaFix = new Line[this.toPlot.size()];
		for(int i=0; i<this.toPlot.size(); i++)
		{
			dumbJavaFix[i] = this.toPlot.get(i);
		}
		LineMap finalMap = new LineMap(dumbJavaFix, bounds);
		try
		{
			finalMap.createSVGFile(filename);
			finalMap.flip().createSVGFile("mapaFlipY.svg");
		}
		catch (Exception e)
		{
			System.out.print("Exception caught: ");
			System.out.println(e.getMessage());
			System.exit(1);
		}
	}

	//public static void main(String[] args)
	//{
	//	RadarMapping r = new RadarMapping();
	//	//int[] a = new int[]{10,11,12,10,11,12,255,255,255,255,40,43,38,39,40,41,40,40,40,40,255,255,255,255,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55,255,255,255,255,255,255,255,255,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,255,255,255,255,255,255,255,55,55,55,55,55,55,55,55,55,55,55,55,55,55,55};
	//	//int[] a = new int[]{22, 22, 22, 22, 22, 22, 22, 76, 23, 22, 22, 22, 22, 0, 22, 22, 22, 80, 79, 255, 255, 255, 255, 255, 39, 39, 0, 39, 38, 37, 37, 36, 36, 36, 37, 36, 35, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 0, 34, 34, 0, 34, 22, 22, 23, 23, 34, 22, 22, 22, 22, 0, 22, 22, 36, 36, 36, 37, 42, 22, 22, 139, 140, 142, 144, 255, 255, 255, 255, 22, 22, 22, 255, 255, 255, 255, 255, 22, 22};
	//	//int[] a = new int[]{22, 22, 22, 22, 22, 22, 22, 255, 255, 255, 22, 22, 22, 0, 22, 22, 22, 22, 22, 255, 255, 255, 255, 255, 255, 255, 0, 255, 255, 255, 80, 80, 80, 79, 79, 79, 79, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 78, 0, 78, 78, 0, 78, 79, 79, 79, 80, 80, 80, 81, 23, 22, 0, 22, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 22, 22, 22, 255, 255, 255, 255, 255, 22, 22, 255};
	//	int[] a = new int[]{101, 101, 101, 101, 102, 107, 107, 108, 108, 108, 108, 108, 108, 109, 109, 109, 109, 110, 110, 110, 111, 255, 255, 255, 255, 44, 43, 43, 42, 42, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 42, 42, 42, 42, 43, 43, 43, 43, 44, 44, 44, 45, 46, 47, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
	//	//int[] a = new int[]{};
	//	ArrayList<Integer> test = new ArrayList<Integer>();
	//	for(int i=0; i<a.length; i++)
	//	{
	//		test.add(a[i]);
	//	}
	//	//r.convertFullScanFromPolar(test);
	//	//r.smartConvertFullScanFromPolar(test);
	//	r.recursiveConvert(test);
	//	r.createLinesNaive();
	//	r.inspectToPlot();
	//	r.plotMap("test.svg");
	//}

}
