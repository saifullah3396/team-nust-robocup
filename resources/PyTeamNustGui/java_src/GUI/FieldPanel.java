package GUI;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.geom.Point2D;
import java.util.Vector;

import javax.swing.JButton;
import javax.swing.JPanel;

import DataContainers.Field;
import DataContainers.FieldMeasure;

public class FieldPanel extends JPanel {

	MainFrame parentFrame;
	FieldMeasure measure;
	Field field;
	
	public Point2D.Double CenterMid = new Point2D.Double();
	public double fieldToScreen = 1;
	double fieldRatio;
	
	boolean gridVisible = false;
	Vector<FieldPaintable> paintObjects = new Vector<FieldPaintable> ();
	/**
	 * Create the panel.
	 */
	public FieldPanel(MainFrame parentFrame) {
		this.parentFrame = parentFrame;
		field = parentFrame.field;
		measure = field.measure;
		
		fieldRatio = measure.carpetDim.getX() / measure.carpetDim.getY();
			

		
	}
	
	Color greenFieldColor = new Color(0, 150, 0);
	
	public void paintComponent(Graphics g) {
	    // Let UI Delegate paint first, which 
	    // includes background filling since 
	    // this component is opaque.
		super.paintComponent(g); 
		
		Graphics2D g2 = (Graphics2D) g;
		paintField(g2);
		
		g2.setColor( Color.BLACK);
		String str1 = "Blackboard";
		
		g2.drawString(str1, this.getWidth() -100,  300);
		g2.drawString("door", this.getWidth() -100,  50);
	    //----------------------------------------
	    //paint other stuff
		for(int j=0; j<paintObjects.size(); j++)
		{
			paintObjects.get(j).paintCustom(g2,  this);
//			Point2D.Double temp = new Point2D.Double(j * 500,0);
//			temp = FieldToScreen(temp);
//			g2.drawOval( (int) temp.getX(), (int) temp.getY(), (int) (20 / fieldToScreen), (int) (20 / fieldToScreen));
		}
	}  
	
	public Point2D.Double FieldToScreen(Point2D.Double pt)
	{
		return new Point2D.Double(CenterMid.getX() + pt.getX() / fieldToScreen, CenterMid.getY() -  pt.getY() / fieldToScreen);
	}
	
	void paintField(Graphics2D g2)
	{
		int width = this.getWidth();
		int height = this.getHeight() - 5;
		Point2D.Double screenSize = new Point2D.Double(width, height);
		
		int margin = 0;
		screenSize.setLocation(screenSize.getX() - margin, screenSize.getY() - margin);
		
		float ratio = (float)(width) / (float)(height);
		
		if(ratio < fieldRatio)
		{
			//width is limiting
			
			fieldToScreen = measure.carpetDim.getX() / width;
	
		}else
		{
			//height is limiting
			fieldToScreen = measure.carpetDim.getY() / height;
			
		}
	
		//System.out.println("Screen is " + screenSize);
		//System.out.println("Ratio is " + Double.toString(fieldToScreen));
		
		
		//----------------------------------------
		//draw carpet
		g2.setColor( greenFieldColor);
		Point2D scaledCarpet = new Point2D.Double(measure.carpetDim.getX() / fieldToScreen, measure.carpetDim.getY() / fieldToScreen);	
		//System.out.println("scaledCarpet will be " + scaledCarpet);
		Point2D.Double carpetDrawCorner = new Point2D.Double (( width - scaledCarpet.getX() ) /2 , ( height - scaledCarpet.getY() ) /2);
		//System.out.println("Margin will be " + carpetCorner);  
	    g2.fillRect((int)carpetDrawCorner.getX(), (int)carpetDrawCorner.getY(), (int) scaledCarpet.getX(),  (int) scaledCarpet.getY());
	    
	    //----------------------------------------
	    //draw boundary lines
	    float LineWidth = (float) (measure.lineWidth / fieldToScreen);
	    g2.setStroke(new BasicStroke(LineWidth));
	    g2.setColor( Color.white);
	    Point2D.Double  FieldDrawCorner = new Point2D.Double ( measure.fieldCarpetMargin / fieldToScreen + carpetDrawCorner.getX(), measure.fieldCarpetMargin / fieldToScreen + carpetDrawCorner.getY());
	    //System.out.println("Field draw corner will be " + FieldDrawCorner);
	    Point2D scaledField = new Point2D.Double(measure.fieldDim.getX() / fieldToScreen, measure.fieldDim.getY() / fieldToScreen);
	    g2.drawRect( (int)FieldDrawCorner.getX(), (int)FieldDrawCorner.getY(), (int) scaledField.getX(),  (int) scaledField.getY());
	    //System.out.println("Field size will be " + scaledField);
	    
	    
	    //----------------------------------------
	    //draw mid line
	    Point2D.Double CenterTop = new Point2D.Double ( FieldDrawCorner.getX() + measure.fieldDim.getX() / 2 / fieldToScreen, FieldDrawCorner.getY());
	    Point2D.Double CenterBottom = new Point2D.Double ( FieldDrawCorner.getX() + measure.fieldDim.getX() / 2 / fieldToScreen, FieldDrawCorner.getY() + measure.fieldDim.getY() / fieldToScreen);
	    //System.out.println("Center line from " + CenterTop + " to " + CenterBottom);
	    g2.drawLine((int)CenterTop.getX(), (int)CenterTop.getY(),(int)CenterBottom.getX(), (int)CenterBottom.getY());
	    
	    //----------------------------------------
	    //draw center mid line
	    CenterMid.setLocation( new Point2D.Double ( CenterTop.getX(), (CenterTop.getY() + CenterBottom.getY())/2) );
	    Point2D.Double CenterMidLeft = new Point2D.Double ( CenterMid.getX() - measure.centerMidLineLength / 2 / fieldToScreen, CenterMid.getY());
	    Point2D.Double CenterMidRight = new Point2D.Double ( CenterMid.getX() + measure.centerMidLineLength / 2 / fieldToScreen, CenterMid.getY());
	    //System.out.println("Center mid line from " + CenterMidLeft + " to " + CenterMidRight);
	    g2.drawLine((int)CenterMidLeft.getX(), (int)CenterMidLeft.getY(),(int)CenterMidRight.getX(), (int)CenterMidRight.getY());
	 	    
	    //----------------------------------------
	    //draw mid circle
	    Point2D.Double CircleLeftTop = new Point2D.Double ( CenterMid.getX() - measure.centerCircleDiameter / 2 / fieldToScreen, CenterMid.getY() - measure.centerCircleDiameter / 2 / fieldToScreen);
	    //System.out.println("Cicle Left Top at " + CircleLeftTop);
	    g2.drawOval( (int) CircleLeftTop.getX(), (int) CircleLeftTop.getY(), (int) (measure.centerCircleDiameter / fieldToScreen), (int) (measure.centerCircleDiameter / fieldToScreen));
	
	    //----------------------------------------
	    //draw penalty areas
	    int penaltyX = (int) (measure.penaltyAreaX / fieldToScreen);
	    int penaltyY = (int) (measure.penaltyAreaY / fieldToScreen);
	    Point2D.Double MiddleLeftLine = new Point2D.Double ( FieldDrawCorner.getX(), CenterMid.getY());
	    Point2D.Double LeftUpPenalty = new Point2D.Double ( MiddleLeftLine.getX(), CenterMid.getY() - penaltyY/2);
	    Point2D.Double LeftDownPenalty = new Point2D.Double ( MiddleLeftLine.getX(), CenterMid.getY() + penaltyY/2);
	    
	    g2.drawLine( (int)LeftUpPenalty.getX(), (int)LeftUpPenalty.getY(), (int)(LeftUpPenalty.getX() + penaltyX) , (int)LeftUpPenalty.getY());
	    g2.drawLine( (int)LeftDownPenalty.getX(), (int)LeftDownPenalty.getY(), (int)(LeftDownPenalty.getX() + penaltyX) , (int)LeftDownPenalty.getY());
	    g2.drawLine( (int)(LeftUpPenalty.getX() + penaltyX), (int)LeftUpPenalty.getY(), (int)(LeftDownPenalty.getX() + penaltyX) , (int)LeftDownPenalty.getY());
	    
	    Point2D.Double MiddleRightLine = new Point2D.Double ( FieldDrawCorner.getX() + measure.fieldDim.getX() / fieldToScreen, CenterMid.getY());
	    
	    Point2D.Double RightUpPenalty = new Point2D.Double ( MiddleRightLine.getX(), CenterMid.getY() - penaltyY/2);
	    Point2D.Double RightDownPenalty = new Point2D.Double ( MiddleRightLine.getX(), CenterMid.getY() + penaltyY/2);
	    
	    g2.drawLine( (int)RightUpPenalty.getX(), (int)RightUpPenalty.getY(), (int)(RightUpPenalty.getX() - penaltyX) , (int)LeftUpPenalty.getY());
	    g2.drawLine( (int)RightDownPenalty.getX(), (int)RightDownPenalty.getY(), (int)(RightDownPenalty.getX() - penaltyX) , (int)RightDownPenalty.getY());
	    g2.drawLine( (int)(RightUpPenalty.getX() - penaltyX), (int)LeftUpPenalty.getY(), (int)(RightDownPenalty.getX() - penaltyX) , (int)RightDownPenalty.getY());


	    //----------------------------------------
	    //draw penalty points
	    Point2D.Double RightPenaltyPoint = new Point2D.Double ( MiddleRightLine.getX() - measure.penaltyPointDistance /fieldToScreen, CenterMid.getY());
	    Point2D.Double LeftPenaltyPoint = new Point2D.Double ( MiddleLeftLine.getX() + measure.penaltyPointDistance /fieldToScreen, CenterMid.getY());
	    
	    int penaltyPointSize = (int) (measure.penaltyPointSize / fieldToScreen / 2);
	    g2.drawLine( (int)LeftPenaltyPoint.getX(), (int)(LeftPenaltyPoint.getY() - penaltyPointSize), (int)LeftPenaltyPoint.getX() , (int)(LeftPenaltyPoint.getY() + penaltyPointSize));
	    g2.drawLine( (int)(LeftPenaltyPoint.getX() - penaltyPointSize), (int)LeftPenaltyPoint.getY(), (int)(LeftPenaltyPoint.getX() + penaltyPointSize) , (int)LeftPenaltyPoint.getY());

	    g2.drawLine( (int)RightPenaltyPoint.getX(), (int)(RightPenaltyPoint.getY() - penaltyPointSize), (int)RightPenaltyPoint.getX() , (int)(RightPenaltyPoint.getY() + penaltyPointSize));
	    g2.drawLine( (int)(RightPenaltyPoint.getX() - penaltyPointSize), (int)RightPenaltyPoint.getY(), (int)(RightPenaltyPoint.getX() + penaltyPointSize) , (int)RightPenaltyPoint.getY());

	    //----------------------------------------
	    //draw goal
	    g2.setColor( Color.LIGHT_GRAY);
	    int GoalPostWidth = (int) (measure.goalWidth / fieldToScreen / 2);
	    int GoalDepth = (int) (measure.goalDepth / fieldToScreen);
	    g2.fillRect( (int)(MiddleLeftLine.getX()  - GoalDepth), (int)(CenterMid.getY() - GoalPostWidth), GoalDepth,  GoalPostWidth*2);
	    g2.fillRect( (int)(MiddleRightLine.getX()), (int)(CenterMid.getY()-GoalPostWidth), GoalDepth,  GoalPostWidth*2);

	    g2.setColor( Color.white);
	    g2.setStroke(new BasicStroke((float) (measure.goalLineThickness / fieldToScreen)));
	    
	    g2.drawLine( (int)(MiddleLeftLine.getX()), (int)(CenterMid.getY() - GoalPostWidth), (int)(MiddleLeftLine.getX()) , (int)(CenterMid.getY() + GoalPostWidth));
	    g2.drawLine( (int)(MiddleRightLine.getX()), (int)(CenterMid.getY() - GoalPostWidth), (int)(MiddleRightLine.getX()) , (int)(CenterMid.getY() + GoalPostWidth));
		
	    //if v need to draw the grid
	    if(gridVisible)
	    {
	    	g2.setColor( Color.LIGHT_GRAY);
	    	g2.setStroke(new BasicStroke((float) ( 5 / fieldToScreen)));
	    	g2.setComposite(makeComposite(0.4f));
		    int gridSize = 500; //100 mm
			double gridSizeOnScreen = gridSize / fieldToScreen;
			
			Point2D.Double FieldBottomLeftCorner = new Point2D.Double (FieldDrawCorner.x, FieldDrawCorner.y + measure.fieldDim.getY() / fieldToScreen);
			Point2D.Double FieldTopRightCorner = new Point2D.Double (FieldDrawCorner.x + measure.fieldDim.getX() / fieldToScreen, FieldDrawCorner.y );
			
			int timesX = (int) ((FieldTopRightCorner.x - FieldDrawCorner.x)/gridSizeOnScreen);
			int timesY = (int) ((FieldBottomLeftCorner.y - FieldDrawCorner.y)/gridSizeOnScreen);
			for(int x=1; x<=timesX; x++)
			{
				 g2.drawLine( (int)(FieldDrawCorner.getX() + gridSizeOnScreen*x), (int)FieldDrawCorner.getY(), (int)(FieldBottomLeftCorner.getX() + gridSizeOnScreen*x) , (int)FieldBottomLeftCorner.getY());
			}
			
			for(int y=1; y<timesY; y++)
			{
				 g2.drawLine( (int)FieldDrawCorner.getX(), (int)(FieldDrawCorner.getY() + gridSizeOnScreen*y), (int)FieldTopRightCorner.getX(), (int)(FieldDrawCorner.getY()+  gridSizeOnScreen*y));
			}
	    }
	    
	    
	    
	    for(int j=0; j<field.robots.size(); j++)
	    {
	    	if( field.robots.get(j).ballPosMap.cells.size() > 0 )
	    	{
	    		field.robots.get(j).ballPosMap.paintCustom(g2, this);
	    		break;
	    	}
	    }
	    
	    //draw a temp robot
	    Point2D.Double position = new Point2D.Double(4400, 300);
	    
//		//draw the robot now
//	    g2.setColor( Color.blue);
//		g2.setStroke(new BasicStroke((float) (10 / fieldToScreen)));
//		g2.setComposite(makeComposite(0.8f));
//		Point2D center = FieldToScreen(position);
//		drawCircle(g2, center, 100, false);
	    
	}


	//drawing functions
	public void drawCircle(Graphics2D g2, Point2D center, int radius, boolean filled )
	{
		double scaledRadius = radius/ fieldToScreen;
		if(!filled)
		{
			g2.drawOval( (int) (center.getX() - scaledRadius), (int) (center.getY() - scaledRadius), (int) (scaledRadius *2 ), (int) (scaledRadius *2 ));
		}else
		{
			g2.fillOval( (int) (center.getX() - scaledRadius), (int) (center.getY() - scaledRadius), (int) (scaledRadius *2 ), (int) (scaledRadius *2 ));
		}
	}
	
	
	public void drawCross(Graphics2D g2, Point2D center, int radius)
	{
		double scaledRadius = radius/ fieldToScreen;

		g2.drawLine( 	(int) (center.getX() - scaledRadius), 
						(int) (center.getY() - scaledRadius), 
						(int) (center.getX() + scaledRadius), 
						(int) (center.getY() + scaledRadius));
		
		g2.drawLine( 	(int) (center.getX() + scaledRadius), 
						(int) (center.getY() - scaledRadius), 
						(int) (center.getX() - scaledRadius), 
						(int) (center.getY() + scaledRadius));		
	}

	public AlphaComposite makeComposite(float alpha) 
	{
		int type = AlphaComposite.SRC_OVER;
		return(AlphaComposite.getInstance(type, alpha));
	}
}
