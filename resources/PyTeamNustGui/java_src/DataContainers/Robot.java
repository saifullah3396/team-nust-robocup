package DataContainers;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.Deque;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Vector;

import GUI.FieldPaintable;
import GUI.FieldPanel;
import GUI.NetworkHandler;

public class Robot implements FieldPaintable{
	public String name;
	public int id;
	
	Field ParentField;
	
	//variables
	public Memory blackboard = new Memory("Blackboard");
	public Memory debugMemory = new Memory("Debug Force");
	
	
	//thread data
	public Vector<ThreadData> threadsData= new Vector<ThreadData>();
	
	//network variables
	NetworkHandler network;
	NetworkHandler imageNetwork;
	String networkName;
	
	//location
	Point2D.Double globalLocation = new Point2D.Double();
	Double globalAngle = (double) 0;
	
	//robot's planned path
	public Vector<Point2D.Double> path = new Vector<Point2D.Double>();
	Point2D.Double currentPathPoint = new Point2D.Double();
	double walkArgAngle = 0;
	double walkMag= 0 ;
	Point2D.Double appPoint = new Point2D.Double();
	Point2D.Double controlPt = new Point2D.Double();
	public Vector<Point2D.Double> EField =  new Vector<Point2D.Double>();
	public Vector<Point2D.Double> EFieldSpecial =  new Vector<Point2D.Double>();
	public Deque<Point2D.Double> positionHistory =  new LinkedList<Point2D.Double>();
	
	//robot ai
	public int role = -1;
	
	//robot maps
	public Map ballPosMap;
	public Map robotPosMap;
	
	//robot localization
	public Vector<PFState> pfStates = new Vector<PFState>();
	
	// Footsteps
	public Vector<FootStep> footSteps = new Vector<FootStep>();
	
	
	//painting data
	RobotPaintData paintData = new RobotPaintData();
	
	public Robot(Field ParentField, int id, String name, String networkName)
	{
		this.ParentField = ParentField;
		this.id = id;
		this.name = name;
		this.networkName = networkName;
		
		ballPosMap = new Map(this);
		updatePaintData();
		
	}
	
	public void connect()
	{
		network = new NetworkHandler(ParentField.ParentFrame, this, ParentField.ParentFrame.logControl);
		network.setConnectionSettings(networkName, 20010, 60000, 5000);
		network.start();
		
		imageNetwork = new NetworkHandler(ParentField.ParentFrame, this, ParentField.ParentFrame.logControl);
		imageNetwork.setConnectionSettings(networkName, 20011, 60000, 5000);
		imageNetwork.keepAliveInterval = 3000;
		imageNetwork.start();
	}
	
	
	public void sendUserCommand(String command)
	{
		network.sendUserCommand(command);
	}
	
	
	public void sendCommand(String command)
	{
		network.sendCommand(command);
	}
	
	public void sendSelectLine(String selectLine)
	{
		network.sendSelectLine(selectLine);
	}
	
	public void OnMemoryValuesUpdate()
	{
		updatePaintData();
		ParentField.updateBallModel();
		
		//signal the field to update its view
		ParentField.ParentFrame.fieldPanel.repaint();		
	}
	
	//---------------------------------------------------------------------------
	//string parsing
	
	public void fillMemoryValuesFromString(Memory memory, String str)
	{
		try
		{
			int ptrStart = 0;
			
			int ptr = str.indexOf('{');
			int endPtr = str.lastIndexOf('}');
			
			String part = str.substring(ptr+1, endPtr);
			
			int divide = part.indexOf(':');
			String numStr = part.substring(0,  divide);
			int num = Integer.parseInt(numStr);
					
			String data = part.substring(divide+1);

			Vector<String> values = new Vector<String>();
			int ptrDeep = 0;
			while(true)
			{		
				if(ptrDeep >= data.length())
				{
					break;
				}
				
				int commaPtr = data.indexOf(',', ptrDeep);
				int bracketPtr = data.indexOf('{', ptrDeep);
				
				if(bracketPtr == -1 || commaPtr < bracketPtr)	//if comma is before bracket
				{
					if(commaPtr != -1)
					{
						values.add(data.substring(ptrDeep, commaPtr));
						ptrDeep = commaPtr+1; 
					}
					else
					{
						values.add(data.substring(ptrDeep));	//get the string till the end
						ptrDeep = data.length(); 
					}
					
				}else
				{
					//bracket is ahead
					int endBracktPtr = data.indexOf('}', bracketPtr);
					values.add(data.substring(bracketPtr, endBracktPtr+1));
					ptrDeep = endBracktPtr+2;
				}
			}
			//System.out.println("NUM and values: ");
			//System.out.println(num);
			//System.out.println(values.size());
			if(values.size() == num && values.size() == memory.variables.size())	//if the parts are equal to the size given
			{
				for(int j=0; j<values.size(); j++)
				{
					memory.variables.get(j).value = values.get(j);
				}
			}
			else
			{
				throw(new Exception());
			}
				
			OnMemoryValuesUpdate();
			
		}catch(Exception ex)
		{
			System.out.println("Corrupted blackboard values data received");
		}
		
		ParentField.updateBallModel();
		ParentField.ParentFrame.updateFieldIfVisible();
		
	}
	
	public void fillMemoryNamesFromString(Memory memory, String str)
	{
		memory.variables.clear();
		try
		{
			int ptrStart = 0;
			
			int ptr = str.indexOf('{');
			int endPtr = str.lastIndexOf('}');
			
			String part = str.substring(ptr+1, endPtr);
			
			String portions[] = part.split(":");
			
			if(portions.length == 2)
			{
				int num = Integer.parseInt( portions[0]);
				
				String[] names = portions[1].split(",");
				
				if(names.length == num)	//if the parts are equal to the size given
				{
					for(int j=0; j<names.length; j++)
					{
						memory.variables.add(new Variable(j, names[j]));
					}
				}
				else
				{
					throw(new Exception());
				}
				
			}
			
		}catch(Exception ex)
		{
			System.out.println("Corrupted blackboard header data received");
		}		
	}
	
	public void createThreadsFromHeader(String str)
	{
		threadsData.clear();
		
		try
		{
			String threadStr[] = str.split("%");
			
			for(int m=0; m<threadStr.length; m++)
			{
				
				threadsData.add(new ThreadData());
				threadsData.lastElement().threadid = threadsData.size()-1;
				String connectorStr[] = threadStr[m].split("&");
				for(int k = 0; k<connectorStr.length; k++)
				{
					int ptrStart = 0;
					
					int ptr = connectorStr[k].indexOf('{');
					int endPtr = connectorStr[k].lastIndexOf('}');
					
					String part = connectorStr[k].substring(ptr+1, endPtr);
					
					String portions[] = part.split(":");
					
					if(portions.length == 2)
					{
						int num = Integer.parseInt( portions[0]);
						
						String[] names = portions[1].split(",");
						
						if(names.length == num)	//if the parts are equal to the size given
						{
							
							
							for(int j=0; j<names.length; j++)
							{
								Variable varBlackboard = findVariableFromName(blackboard, names[j]);
								Variable varDebug = findVariableFromName(debugMemory, names[j]);
								if(varBlackboard != null && varDebug != null)
								{
									if(k == 0)//input connector
									{
										threadsData.lastElement().inputConnector.variables.add(varBlackboard);
										threadsData.lastElement().inputConnector.forceVariable.add(varDebug);
										threadsData.lastElement().inputConnector.selectLines.add(0);
									}else	//output connector
									{
										threadsData.lastElement().outputConnector.variables.add(varBlackboard);
									}
								}else
								{
									throw(new Exception());
								}
								
							}
						}
						else
						{
							throw(new Exception());
						}
						
					}
				}
			}
		}catch(Exception ex)
		{
			System.out.println("Corrupted thread header data received");
		}
		// System.out.println(threadsData.size());
	}
	
	public void setThreadConnectorSelectLines(String str)
	{		
		try
		{
			String threadStr[] = str.split("%");
			
			for(int m=0; m<threadStr.length; m++)
			{
				
				int ptrStart = 0;
				
				int ptr = threadStr[m].indexOf('{');
				int endPtr = threadStr[m].lastIndexOf('}');
				
				String part = threadStr[m].substring(ptr+1, endPtr);
				
				String portions[] = part.split(":");
				
				if(portions.length == 2)
				{
					int num = Integer.parseInt( portions[0]);
					
					String[] values = portions[1].split(",");
					
					if(values.length == num)	//if the parts are equal to the size given
					{
						for(int j=0; j<values.length; j++)
						{
							threadsData.get(m).inputConnector.selectLines.set(j, Integer.parseInt(values[j]));
						}
					}
					else
					{
						throw(new Exception());
					}
					
				}
			}

		}catch(Exception ex)
		{
			System.out.println("Corrupted thread select lines data received");
		}
	}
	
	Variable findVariableFromName(Memory memory, String name)
	{
		for(int j=0; j<memory.variables.size(); j++)
		{
			if(memory.variables.get(j).name.equals(name))
			{
				return memory.variables.get(j);
			}
		}
		
		return null;
	}
	
	//-----------------------------------------------------------------------------
	//painting functions
	int robotLocationIndex = -1;
	int robotAngleIndex = -1;
	int currentPathPointIndex = -1;
	int walkArgumentsIndex = -1;
	int appPointIndex = -1;
	int pathControlPointWorldIndex = -1;

	void updatePaintData()
	{
		
		if(robotLocationIndex == -1)
		{
			for(int j=0; j<blackboard.variables.size(); j++)
			{
				if(blackboard.variables.get(j).name.equals("robotPose2D") )
				{
					robotLocationIndex = j;
					break;
				}
			}
		
		}
		
		if(robotLocationIndex != -1)
		{
			String loctionStr = blackboard.variables.get(robotLocationIndex).value;
			
			Vector<Double> location = Utilities.ParseStringToVector(loctionStr);
			
			
			paintData.position.setLocation( location.get(0) * 1000, location.get(1) * 1000 );
			paintData.angle = location.get(2);
			
//			switch(id)
//			{
//			case 0:
//			{
//				paintData.position.setLocation( 0, 0 );
//				break;
//			}
//			case 1:
//			{
//				paintData.position.setLocation( 500, 500 );
//				break;
//			}
//			case 2:
//			{
//				paintData.position.setLocation( 500, -500 );
//				break;
//			}
//			case 3:
//			{
//				paintData.position.setLocation( 4500, 0000 );
//				break;
//			}
//			case 4:
//			{
//				paintData.position.setLocation( 3000, 1000 );
//				break;
//			}
//			case 5:
//			{
//				paintData.position.setLocation( 3500, -1000 );
//				break;
//			}
//			
//			}
//			paintData.angle = 0;
			
			globalLocation = paintData.position;
			globalAngle = paintData.angle ;
			
			
			synchronized(positionHistory)
			{
				//System.out.println("before " + positionHistory);
				
				Point2D.Double temp = new Point2D.Double(globalLocation.x, globalLocation.y);
				
				Point2D.Double last = positionHistory.peekLast();
				Point2D.Double diff = null;
				if(last!= null)
				{
					diff = new Point2D.Double(last.x - temp.x, last.y - temp.y);
				}
				
				//if(last== null ||  (diff.x*diff.x + diff.y+diff.y) > 20 )
				{
					if(positionHistory.size() > 100)
					{
						positionHistory.remove();
					}
					positionHistory.add(temp);
				}
				
				
				//System.out.println("after " + positionHistory);
			}
			
			//System.out.println(positionHistory);
		}
		
		//current path on points value
		if(currentPathPointIndex == -1)
		{
			for(int j=0; j<blackboard.variables.size(); j++)
			{
				if(blackboard.variables.get(j).name.equals("CurrentPathPoint") )
				{
					currentPathPointIndex = j;
					break;
				}
			}
		
		}else
		{
			String currentPathStr = blackboard.variables.get(currentPathPointIndex).value;
			Vector<Double> loc = Utilities.ParseStringToVector(currentPathStr);
			currentPathPoint.setLocation( loc.get(0), loc.get(1) );		
		}
		
		//walk arguments on robott
		if(walkArgumentsIndex == -1)
		{
			for(int j=0; j<blackboard.variables.size(); j++)
			{
				if(blackboard.variables.get(j).name.equals("WalkArguments") )
				{
					walkArgumentsIndex = j;
					break;
				}
			}
		
		}else
		{
			String walkArgsString = blackboard.variables.get(walkArgumentsIndex).value;
			Vector<Double> walkArgs = Utilities.ParseStringToVector(walkArgsString);
			
			walkMag = Math.sqrt( walkArgs.get(1)*walkArgs.get(1) + walkArgs.get(0)*walkArgs.get(0) );
			walkArgAngle = walkArgs.get(2);
		}
		
		//walk arguments on robott
		if(appPointIndex == -1)
		{
			for(int j=0; j<blackboard.variables.size(); j++)
			{
				if(blackboard.variables.get(j).name.equals("ApproachPointWorld") )
				{
					appPointIndex = j;
					break;
				}
			}
		
		}else
		{
			String appPointString = blackboard.variables.get(appPointIndex).value;
			Vector<Double> appPointVec = Utilities.ParseStringToVector(appPointString);
			
			appPoint.setLocation( appPointVec.get(0), appPointVec.get(1));
		}
		
		//walk arguments on robott
		if(pathControlPointWorldIndex == -1)
		{
			for(int j=0; j<blackboard.variables.size(); j++)
			{
				if(blackboard.variables.get(j).name.equals("PathControlPointWorld") )
				{
					pathControlPointWorldIndex = j;
					break;
				}
			}
		
		}else
		{
			String controlPtString = blackboard.variables.get(pathControlPointWorldIndex).value;
			Vector<Double> controlPtVec = Utilities.ParseStringToVector(controlPtString);
			
			controlPt.setLocation( controlPtVec.get(0), controlPtVec.get(1));
		}
		
		
	}

	@Override
	public void paintCustom(Graphics2D g2, FieldPanel fieldPanel) {
		// TODO Auto-generated method stub

		synchronized(pfStates)
		{	
			for(int j=0; j<pfStates.size(); j++)
			{
				PFState currentState = pfStates.get(j);
				
				g2.setComposite(fieldPanel.makeComposite(0.7f));
				
				//colorIndex = 10;
				g2.setColor(Color.black);
				
				//System.out.println(cells.get(j));

				Point2D.Double center = fieldPanel.FieldToScreen((java.awt.geom.Point2D.Double) currentState.pos);
				//System.out.println(center.toString() );
				fieldPanel.drawCircle(g2, 
						center, 
						50, 
						true);
				
			}
		}	
		
		if(path.size() > 1)
		{
			g2.setColor( Color.red);
			g2.setStroke(new BasicStroke((float) (pathThickness / fieldPanel.fieldToScreen)));
			g2.setComposite(fieldPanel.makeComposite(0.6f));
			//draw the robot path first
			for(int j=0; j<path.size()-1; j++)
			{		
				if(j == 1)
				{
					g2.setColor( Color.magenta);
				}
				Point2D.Double p1 = fieldPanel.FieldToScreen( path.get(j) );
				Point2D.Double p2 = fieldPanel.FieldToScreen( path.get(j+1) );
				g2.drawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y);
			}
			
			
			
			//draw the current pointon robot's path
			g2.setColor( Color.CYAN);
			g2.setStroke(new BasicStroke((float) (pathPtThickness / fieldPanel.fieldToScreen)));
			g2.setComposite(fieldPanel.makeComposite(0.8f));
			Point2D currentPathPtScreen = fieldPanel.FieldToScreen( currentPathPoint);
			fieldPanel.drawCircle(g2, currentPathPtScreen, pathPtRadius, false);
		
			
			//draw the approach point on robot's path
			g2.setColor( Color.blue);
			g2.setStroke(new BasicStroke((float) (20 / fieldPanel.fieldToScreen)));
			g2.setComposite(fieldPanel.makeComposite(0.8f));
			Point2D appPointScreen = fieldPanel.FieldToScreen( appPoint);
			fieldPanel.drawCircle(g2, appPointScreen, 10, false);
			
	
			//draw the control point for bezeir curve on robot's path
			g2.setColor( Color.YELLOW);
			g2.setStroke(new BasicStroke((float) (pathPtThickness / fieldPanel.fieldToScreen)));
			g2.setComposite(fieldPanel.makeComposite(0.8f));
			Point2D controlPtScreen = fieldPanel.FieldToScreen( controlPt);
			fieldPanel.drawCircle(g2, controlPtScreen, pathPtRadius, false);
		}
		
		//draw robot traced path
		synchronized(positionHistory)
		{
			if(positionHistory.size() > 3)
			{
				g2.setColor( Color.BLUE);
				g2.setStroke(new BasicStroke((float) (robotCircleThickness / fieldPanel.fieldToScreen)));
				Point2D.Double lastPt = null;
				
				
				for(int j=0; j< positionHistory.size(); j++)
				{
					Point2D.Double currentPt = positionHistory.poll();
					positionHistory.add(currentPt);
					
					currentPt = fieldPanel.FieldToScreen( currentPt);
					if(lastPt != null)
					{
						//System.out.println(Integer.toString(j));
						
						//draw it
						//System.out.println("drwaing line");
						g2.drawLine( 
								(int)lastPt.getX(), 
								(int)lastPt.getY(), 
								(int)(currentPt.getX()), 
								(int)(currentPt.getY() ));
					}
					
					lastPt = currentPt;	//for next time
				}
			}
		}
		
		//draw the robot now
		switch(role)
		{
		case 0: 	g2.setColor( Color.blue); break;	//support
		case 1: 	g2.setColor( Color.yellow); break;	//pass send
		case 2: 	g2.setColor( Color.BLACK); break;	//shooting
		case 3: 	g2.setColor( Color.red); break;	//idle
		default: g2.setColor( Color.yellow); break;	//support
		}
		g2.setStroke(new BasicStroke((float) (robotCircleThickness / fieldPanel.fieldToScreen)));
		g2.setComposite(fieldPanel.makeComposite(0.8f));
		Point2D center = fieldPanel.FieldToScreen(paintData.position);
		fieldPanel.drawCircle(g2, center, robotCircleRadius, false);
		
		
		//draw the robot's walk arguments	-------------------------
		g2.setColor( Color.DARK_GRAY);
		g2.setStroke(new BasicStroke((float) (6 / fieldPanel.fieldToScreen)));
		g2.setComposite(fieldPanel.makeComposite(1.0f));
		Point2D.Double walkArgsEndPt = Utilities.PolarToCartesian(  walkMag / fieldPanel.fieldToScreen, walkArgAngle + paintData.angle );
		g2.drawLine( 
				(int)center.getX(), 
				(int)center.getY(), 
				(int)(center.getX() + walkArgsEndPt.getX()), 
				(int)(center.getY() - walkArgsEndPt.getY()));
		
		//draw the robot's E field direction-----------------------
		if(EField.size() > 0)
		{
			g2.setColor( Color.PINK);
			g2.setStroke(new BasicStroke((float) (5 / fieldPanel.fieldToScreen)));
			g2.setComposite(fieldPanel.makeComposite(1.0f));
			//draw them all..
			for(int j=0; j<EField.size(); j=j+2)
			{
				
				Point2D pt1 = fieldPanel.FieldToScreen(EField.get(j));
				Point2D pt2 = fieldPanel.FieldToScreen(EField.get(j+1));
				g2.drawLine( 
						(int)pt1.getX(), 
						(int)pt1.getY(), 
						(int)(pt2.getX()), 
						(int)(pt2.getY()) );		
			}
			//draw the starting black dots
			g2.setColor( Color.BLACK);
			g2.setStroke(new BasicStroke((float) (20 / fieldPanel.fieldToScreen)));
			for(int j=0; j<EField.size(); j=j+2)
			{
				Point2D pt1 = fieldPanel.FieldToScreen(EField.get(j));
				g2.drawLine( 
						(int)pt1.getX(), 
						(int)pt1.getY(), 
						(int)(pt1.getX()), 
						(int)(pt1.getY()) );	
			}
			
			
			g2.setStroke(new BasicStroke((float) (50 / fieldPanel.fieldToScreen)));
			for(int j=0; j<EFieldSpecial.size(); j++)
			{
				Point2D.Double pt = fieldPanel.FieldToScreen(EFieldSpecial.get(j));	//way point
			
				switch(j)
				{		
				case 0:
					g2.setColor( Color.MAGENTA);
					break;
				case 1:
					g2.setColor( Color.BLACK);
					break;
				default:
					g2.setColor( Color.yellow);
					break;
				}
			
				switch(j)
				{
				case 1:
					fieldPanel.drawCross(g2, pt, 50);	//goal point
					break;
				default:
					g2.drawLine( 	//everuything else is dot
							(int)pt.getX(), 
							(int)pt.getY(), 
							(int)(pt.getX()), 
							(int)(pt.getY()) );	
				}
			}
			
		}
		
		g2.setColor( Color.BLACK);
		g2.setStroke(new BasicStroke((float) (20 / fieldPanel.fieldToScreen)));
		Point2D lineEnd = Utilities.PolarToCartesian(robotDirectionPointerLength / fieldPanel.fieldToScreen, paintData.angle);
		g2.drawLine( 
				(int)center.getX(), 
				(int)center.getY(), 
				(int)(center.getX() + lineEnd.getX()), 
				(int)(center.getY() - lineEnd.getY()));
		
		g2.setComposite(fieldPanel.makeComposite(0.4f));
		fieldPanel.drawCircle(g2, center, paintData.positionUncertainityRadius, true);
		
		//draw the robot footsteps
		g2.setColor( Color.WHITE);
		g2.setStroke(new BasicStroke((float) (5 / fieldPanel.fieldToScreen)));
		g2.setComposite(fieldPanel.makeComposite(0.6f));
		//draw the robot path first
		Point2D.Double fcenter = fieldPanel.FieldToScreen((java.awt.geom.Point2D.Double) new Point2D.Double(0.0, 0.0));
		//System.out.println("Screen center");
		//System.out.println(fcenter);
		//fieldPanel.drawCross(g2, fcenter, 50);
		for(int j=0; j<footSteps.size(); j++)
		{	
			g2.setColor(new Color(255,0,0));
			FootStep fs = footSteps.get(j);
			Point2D.Double p1 = fieldPanel.FieldToScreen((java.awt.geom.Point2D.Double) fs.p1);
			Point2D.Double p2 = fieldPanel.FieldToScreen((java.awt.geom.Point2D.Double) fs.p2);
			Point2D.Double p3 = fieldPanel.FieldToScreen((java.awt.geom.Point2D.Double) fs.p3);
			Point2D.Double p4 = fieldPanel.FieldToScreen((java.awt.geom.Point2D.Double) fs.p4);
			g2.drawLine((int)p1.x, (int)p1.y, (int)p2.x, (int)p2.y);
			g2.drawLine((int)p2.x, (int)p2.y, (int)p3.x, (int)p3.y);
			g2.drawLine((int)p3.x, (int)p3.y, (int)p4.x, (int)p4.y);
			g2.drawLine((int)p4.x, (int)p4.y, (int)p1.x, (int)p1.y);
		}
		
	}
	
	
	//gui variables
	int robotCircleRadius = 100;
	int robotCircleThickness = 20;
	int robotDirectionPointerLength = 150;
	int pathThickness = 50;
	int pathPtThickness = 20;
	int pathPtRadius = 60;
	
	int walkArgsThickness = 60;
	
	
	public void setPFStateFromBytes(float[] arr)
	{
		if(arr.length % 4 == 0)
		{
			synchronized(pfStates)
			{
				//correct packet
				int numState = arr.length / 4;
				pfStates.clear();
				pfStates.setSize(numState);
				
				for(int j=0; j<numState; j++)
				{
					int ptr = j*4;
					pfStates.set(j, new PFState( new Point2D.Double(arr[ptr] * 1000, arr[ptr+1] * 1000), arr[ptr+2], arr[ptr+3]));
					
					//System.out.println(pfStates.get(j).pos);
				}
			}
		}
	}
	
	public void setFootStepsFromBytes(float[] arr)
	{
		if(arr.length % 8 == 0)
		{
			synchronized(footSteps)
			{
				//correct packet
				int numState = arr.length / 8;
				footSteps.clear();
				footSteps.setSize(numState);
				
				for(int j=0; j<numState; j++)
				{
					int ptr = j*8;
					footSteps.set(j, new FootStep( new Point2D.Double(arr[ptr] * 1000, arr[ptr+1] * 1000), 
												   new Point2D.Double(arr[ptr+2] * 1000, arr[ptr+3] * 1000),
					   							   new Point2D.Double(arr[ptr+4] * 1000, arr[ptr+5] * 1000),
												   new Point2D.Double(arr[ptr+6] * 1000, arr[ptr+7] * 1000)));
					
					//System.out.println(pfStates.get(j).pos);
				}
			}
		}
	}
	
}



class RobotPaintData
{
	public Point2D.Double position = new Point2D.Double();
	public double angle;
	public int positionUncertainityRadius = 10;
}

class FootStep {
	FootStep(
		Point2D.Double p1, 
		Point2D.Double p2,
		Point2D.Double p3,
		Point2D.Double p4
	) 
	{
		this.p1 = p1;
		this.p2 = p2;
		this.p3 = p3;
		this.p4 = p4;
	}
	
	Point2D.Double p1;
	Point2D.Double p2;
	Point2D.Double p3;
	Point2D.Double p4;
}

class PFState{
	Point2D.Double pos;
	float angle;
	float weight;
	
	PFState(Point2D.Double pos,
	float angle,
	float weight)
	{
		this.pos = pos;
		this.angle = angle;
		this.weight = weight;
	}
}