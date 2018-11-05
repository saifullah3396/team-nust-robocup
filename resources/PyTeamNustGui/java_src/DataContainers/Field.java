package DataContainers;

import java.awt.geom.Point2D;
import java.util.NoSuchElementException;
import java.util.Vector;

//import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import GUI.MainFrame;

public class Field {
	public Vector<Robot> robots = new Vector<Robot>();
	MainFrame ParentFrame;
	
	//field dimensions
	public  FieldMeasure measure = new FieldMeasure();
	
	//ball model
	public Ball ball = new Ball();
	
	public Field(MainFrame ParentFrame)
	{
		this.ParentFrame = ParentFrame;
		createRobots();
		
		updateBallModel();
	}
	
	void createRobots()
	{
		Robot robot = null;
		//Robot robot1 = new Robot(this, robots.size(), "Virtual Robot", "localhost");	//create a robot
		//Robot robot1 = new Robot(this, robots.size(), "Virtual Robot", "10.0.30.5");	//create a robot
		robot = new Robot(this, robots.size(), "NAO 4", "192.168.30.5");	//create a robot
		robots.add(robot);
		//robot = new Robot(this, robots.size(),"NAO 2", "192.168.30.4");	//create a robot
		//robots.add(robot);
		/*robot = new Robot(this, robots.size(),"NAO 3", "192.168.1.113");	//create a robot
		robots.add(robot);
		robot = new Robot(this, robots.size(),"NAO 4", "192.168.1.21");	//create a robot
		robots.add(robot);
		robot = new Robot(this, robots.size(),"NAO 5", "192.168.1.115");	//create a robot
		robots.add(robot);
		robot = new Robot(this, robots.size(),"NAO 6", "192.168.1.116");	//create a robot
		robots.add(robot);
		robot = new Robot(this, robots.size(),"NAO 7", "192.168.1.117");	//create a robot
		robots.add(robot);*/
		robot = new Robot(this, robots.size(),"Virtual", "localhost");	//create a robot
		robots.add(robot);

	}
	
	public void connectRobots()
	{
		for(int j=0; j<robots.size(); j++)
		{
			robots.get(j).connect();
		}
	}
	
	int ballLocationIndex = -1;
	public void updateBallModel()
	{
		if(robots.size() > 0)
		{	
			if(ballLocationIndex == -1)
			{
				for(int k=0; k<robots.size(); k++)
				{
					for(int j=0; j<robots.get(k).blackboard.variables.size(); j++)
					{
						if(robots.get(k).blackboard.variables.get(j).name.equals("ballInfo") )
						{
							ballLocationIndex = j;
							break;
						}
					}
					
					if(ballLocationIndex != -1)
					{
						break;
					}
				}
			
			}else
			{
				boolean searchingActiveBot = true;
				boolean foundBall = false;
				
				
				for(int j=0; j<robots.size(); j++)
				{
					if(robots.get(j).blackboard.variables.size() > ballLocationIndex)
					{
						
						if(searchingActiveBot)
						{
							
							
							if(robots.get(j).role == 1 || robots.get(j).role == 2)
							{
								//this is an active bot
							}else
							{
								if(j == robots.size() -1 && searchingActiveBot)
								{
									//the last time
									searchingActiveBot = false;
									j = -1;
								}
								continue;
							}
						}
							
						String strVal = robots.get(j).blackboard.variables.get(ballLocationIndex).value;
						//System.out.println(strVal);
						if(strVal != null)
						{
							Vector<Double> ballLoc = Utilities.ParseStringToVector(strVal);
							if( ballLoc.get(0) == 0 )
							{
								foundBall = true;
								//System.out.println(strVal);
								ball.pos = new Point2D.Double( ballLoc.get(2), ballLoc.get(3) );
								
								synchronized(ball.history)
								{
									ball.history.offerLast(ball.pos);
									
									try{
										if(ball.history.size() > 1000)
										{
											ball.history.removeFirst();
										}
									}catch( NoSuchElementException e)
									{
									
									}
								}
									
							}
							
							if(foundBall)
							{
							
								break;
								
							}
						}
					}
					
					
					if(j == robots.size() -1 && searchingActiveBot)
					{
						//the last time
						searchingActiveBot = false;
						j = -1;
					}
					
				}
				
				
				
				if(!foundBall)
				{
					//no robot knows where the ball is
					ball.pos.setLocation( 13000,7000 );
				}
			}
		}else
		{
			//no data recieved yet
			ball.pos.setLocation( 13000,7000 );
		}
//		ball.pos.setLocation( 1000,00 );
//		synchronized(ball.history)
//		{
//			ball.history.clear();
//		}
		ball.posUncertainityRadius = 0;
		ball.velocity.setLocation(0, 0);
	}

}
