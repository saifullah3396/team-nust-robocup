package DataContainers;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.Point2D;
import java.text.DecimalFormat;
import java.util.Collections;
import java.util.Vector;

import GUI.FieldPaintable;
import GUI.FieldPanel;

public class Map implements FieldPaintable{

	Robot parentRobot;
	public Vector<Double>  cells = new Vector<Double>();	
	Vector<Point2D>  cellCenters = new Vector<Point2D>();
	
	Point dimensions = new Point();
	
	//painting stuf-----------------------
	Vector<Color> colors = new Vector<Color>();
	int colorNum = 200;
	int rangeMax = 1;
	
	Map (Robot parentRobot)
	{
		this.parentRobot = parentRobot;
		
		
		float step = 255 / (float)(colorNum);
		for(int j=0; j<colorNum; j++)
		{
			/*
			 * 	Red 	255, 	0, 		0
				Yellow 	255, 	255, 	0
				Green 	0,		255,	0
			 */
			//System.out.println(step*j);
			//colors.add(new Color(255, (int)step*j, 0));
			colors.add(new Color((int)step*j, (int)step*j, (int)step*j));
		}
	}
	
	public void setCentersFromString(String str)
	{
		//System.out.println("Header=" + str);
		System.out.println("Header");
		if(!str.equals("x"))
		{				
			String portions[] = str.split(";");
			if(portions.length == 2)
			{
				synchronized(cells)
				{
					synchronized(cellCenters)
					{
						cellCenters.clear();
						
						Vector<Double> dim = Utilities.ParseStringToVector(portions[0]);
						
						dimensions.setLocation(dim.get(0),  dim.get(0));
						String points[] = portions[1].split(":");
						cellCenters.setSize(points.length);
						
						
					
						for(int j=0; j<cellCenters.size(); j++)
						{
							//System.out.println(points[j]);
							Vector<Double> point = Utilities.ParseStringToVector(points[j]);
							if(point != null)
							{
								cellCenters.set(j, new Point2D.Double(point.get(0), point.get(1)) ) ;
							}
						}
						
						//System.out.println( cellCenters.toString());
					}
				}
			}

			
		}
		
	}
	
	public void setFromFloatArray(float[] arr)
	{
		synchronized(cells)
		{
			cells.setSize(arr.length);
			int j=0;
			for (float f : arr)
			{
				if( f > rangeMax)
				{
					f = rangeMax;
				}else if (f < -rangeMax)
 				{
					f = -rangeMax;		
				}
				cells.setElementAt((double) f / (double) (rangeMax),  j++);
				
			}
		}
	}
	
	public void setFromNetworkString(String str)
	{
		
		if(!str.equals("x"))
		{		 
			System.out.println("Map=" + str);
			//System.out.println("got map data");
			synchronized(cells)
			{
				cells = Utilities.ParseStringToVector(str);
				
				Double max = Collections.max(cells);
				Double min = Collections.min(cells);
				
				if (min < 0)
				{
					min = -min;
				}
				
				Double scaleFactor = min;
				if(max > min)
				{
					scaleFactor = max;
				}
				
				for (int i=0; i<cells.size(); i++) 
				{
					cells.set(i, cells.get(i) / scaleFactor );
				}
			}

			
		}

	}
	
	
	DecimalFormat df = new DecimalFormat("#.#");
	Font numFont = new Font("TimesRoman", Font.PLAIN, 8); 
	@Override
	public void paintCustom(Graphics2D g2, FieldPanel FieldPanel) {
		// TODO Auto-generated method stub
		g2.setStroke(new BasicStroke((float) (60 / FieldPanel.fieldToScreen)));
		
		
		synchronized(cells)
		{
			synchronized(cellCenters)
			{
				for(int j=0; j<cellCenters.size() && j<cells.size(); j++)
				{					
					
					String str = df.format(cells.get(j));

					if(!str.equals("0") && !str.equals("-0"))
					{
										
						g2.setComposite(FieldPanel.makeComposite(0.7f));
						
						int colorIndex =   (int) (cells.get(j) * (colorNum/2-1));
						colorIndex = colorIndex + colorNum/2;
						
						//colorIndex = 10;
						g2.setColor(colors.get(colorIndex));
						
						//System.out.println(cells.get(j));
	
						Point2D.Double center = FieldPanel.FieldToScreen((java.awt.geom.Point2D.Double) cellCenters.get(j));
						//System.out.println(center.toString() );
						FieldPanel.drawCircle(g2, 
								center, 
								100, 
								true);
						
						Point2D.Double startText = FieldPanel.FieldToScreen(new Point2D.Double(cellCenters.get(j).getX()-50, cellCenters.get(j).getY()-50 ));
						g2.setColor(Color.BLACK);
						g2.setFont(numFont); 
						g2.drawString(str, (int)startText.x, (int)startText.y);
					}
				}
				//System.out.println(cellCenters.size());
			}
		}
		
		
	}

}
