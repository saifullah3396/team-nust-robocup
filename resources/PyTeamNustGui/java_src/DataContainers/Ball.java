package DataContainers;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.LinkedList;
import java.util.NoSuchElementException;
import java.util.Vector;

import GUI.FieldPaintable;
import GUI.FieldPanel;

public class Ball implements FieldPaintable{
	Point2D.Double pos = new Point2D.Double();
	double posUncertainityRadius;
	Point2D.Double velocity = new Point2D.Double();	//has direction in it
	
	LinkedList<Point2D.Double> history = new LinkedList<Point2D.Double>();

	
	@Override
	public void paintCustom(Graphics2D g2, FieldPanel fieldPanel) {
		// TODO Auto-generated method stub
		
		g2.setColor( Color.BLACK);
		g2.setStroke(new BasicStroke((float) (ballHistoryCircleThickness / fieldPanel.fieldToScreen)));
		
		synchronized(history)
		{
			for(int j=0; j<history.size(); j++)
			{
	//			Point2D.Double temp = history.removeFirst();
	//			history.offerLast(temp);
				try
				{
					Point2D center = fieldPanel.FieldToScreen(history.get(j));
					fieldPanel.drawCircle(g2, center, ballHistoryCircleRadius, true);
				}catch(Exception e)
				{
					
				}
				
				
			}
		}
			
		g2.setColor( Color.black);
		g2.setStroke(new BasicStroke((float) (ballCircleThickness / fieldPanel.fieldToScreen)));
		Point2D center = fieldPanel.FieldToScreen(pos);
		fieldPanel.drawCircle(g2, center, ballCircleRadius, true);
		
		Point2D.Double scaledVelocity = new Point2D.Double( velocity.x / fieldPanel.fieldToScreen, velocity.y / fieldPanel.fieldToScreen);
		
		g2.drawLine( 
				(int)center.getX(), 
				(int)center.getY(), 
				(int)(center.getX() + scaledVelocity.getX()), 
				(int)(center.getY() - scaledVelocity.getY()));
		
		g2.setComposite(fieldPanel.makeComposite(0.2f));
		fieldPanel.drawCircle(g2, center, (int)posUncertainityRadius, true);
	}
	
	
	//gui variables
	int ballCircleRadius = 50;
	int ballCircleThickness = 10;
	int ballHistoryCircleThickness = 5;
	int ballHistoryCircleRadius = 20;
	
}
