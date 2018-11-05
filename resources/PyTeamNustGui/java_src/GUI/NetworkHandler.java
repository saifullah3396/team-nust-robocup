package GUI;

import java.awt.geom.Point2D;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Vector;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfFloat;
import org.opencv.highgui.Highgui;

import DataContainers.Robot;
import DataContainers.Utilities;
import LogListController.LogLevelHandler;
import LogListController.LogObject;


//Permission class to pause and resume thread//
/*** Sending Queue Task ***/    
class Permission
{
	private boolean go;
	Permission(boolean GO)
	{
		go = GO;
	}
	void allow()
	{
		go = true;
	}
	void deny()
	{
		go = false;
	}
	
	boolean isAllowed()
	{
		return go;
	}
}
//-permission class ends here---------------------//



//Sending Queue data and classes//
/*** Sending Queue Task ***/    
class SendTask {
    public int priority;
    public String data;
    SendTask(int Priority, String Data)
    {
    	priority = Priority;
    	data = Data;
    }
}

/*** Comparator ***/        
class TaskQueueComparator implements Comparator<SendTask> {

public int compare(SendTask task1, SendTask task2) {
    return task2.priority - task1.priority;
    }        
}
//-Priority Queue stuff ends here---------------------//


/**
 * ------------------------------------------------------------
 * **/
//main communication handle
//handles everything relating to server connection
public class NetworkHandler extends Thread{
	//variables
	Socket socket;
	
	MainFrame parentJFrame;
	Robot parentRobot;
	
	LogLevelHandler logControl;
		
	//communication threads
	private Thread recvThread;
	private Thread sendThread;
	
	//connection options
	private int socketTimeout;
	private String serverName;
	private int port;
	private int connRetryInterval;
	
	//remember if v r connected
	private AtomicBoolean connected = new AtomicBoolean(false);
	private AtomicLong lastKeepAlive = new AtomicLong(0);
	public int keepAliveInterval = 30000;
	int keepAliveMissingAllowed = 30;
	
	
	//for marking startup operations
	boolean startup = true;

	//Data send Queue
	// DO NOT change queue element priority after insertion
	public Queue<SendTask> sendQueue = new PriorityQueue<SendTask>(10, new TaskQueueComparator());
	
	//GUI update operation queue
	//OperationQueue operationQueue;
	
	//comm specific details
	DateTime lastRealtimeUpdate = new DateTime();
	int realtimeUpdateInterval = 2000;
	
	//constructor
	public NetworkHandler(MainFrame parentJFrame,
			Robot parentRobot,
			LogLevelHandler logControl )
			//OperationQueue operationQueue)
	{
		this.parentRobot = parentRobot;
		this.parentJFrame = parentJFrame;
		///this.operationQueue = operationQueue;
		this.logControl = logControl;
	}
	
	//authenticate a newly connected client
	private void authenticateClient()
	{
		if(startup)
		{
			startup = false;
		}
	}
	
	public void run()
	{
		int counter = 0;
		//do this forever
		while(true)
		{
			if(!connected.get())
			{
				//stop everything if there was anything
				
				try {
					disconnect();
					connect();
					connected.set( true );
					lastKeepAlive.set( System.currentTimeMillis( ) );		
					displayInLog(10, "socket is now connected");
				} catch (IOException e) {
					// TODO Auto-generated catch block
					//e.printStackTrace();
					displayInLog(35, "Error connecting");
					displayInLog(35, e.getMessage());
				}	
				
			}else
			{
				//send a keep alive and resend messages which were not acknowledged
				//this will change the connected thing in future

				if( (lastKeepAlive.get() - System.currentTimeMillis( )) > keepAliveInterval * keepAliveMissingAllowed )
				{
					//there is some problem, last 3 keep alives were not replied back
					connected.set( false );
					disconnect();
				}else
				{
					//all is well, send next keep alive
//					String str = "";
//					for(int j=0; j<1000; j++)
//					{
//						str = str + "alive" + Integer.toString(counter) + "\t";
//					}
					sendStringToServer(2, 1, "alive" + Integer.toString(counter));
					//sendStringToServer(2, 1, str);
					//check if there is an outstanding process which has taken too long
					//checkStaleNetworkOperations();
				}
				
			
			}
			counter++;
			//wait for some time and go again
			try {
	  		    Thread.sleep(keepAliveInterval);
	  		} catch(InterruptedException ex) {
	  		    Thread.currentThread().interrupt();
	  		}
		}
	}
	
	//connect and make the send receive threads
	private void connect() throws IOException
	{   
	    //try {
	    	socket = new Socket(serverName, port);
		 //}
		// catch (IOException e) {
		//     System.out.println(e);
		// }

		recvThread = new Thread (new RecvThread(this));
		sendThread = new Thread (new SendThread(this));
		
		recvThread.start();
		sendThread.start();
		
		authenticateClient();

	}
	
	//disconnect and stop evertyhing
	private void disconnect()
	{
		displayInLog(10, "disconnecting socket");
		cleanup(1);
		if(socket != null)
		{
			try {
				socket.close();
				socket = null;
			} catch (IOException e) {
				// TODO Auto-generated catch block
				//e.printStackTrace();
				displayInLog(5, "Error Closing Socket");
			}
		}
		
		sendThread = null;
		recvThread = null;
	}

	//signal the sending thread to send some data
	void addDataToSendQueue(int priority, String data)
	{
		synchronized (sendQueue)
		{
			sendQueue.add(new SendTask(priority, data));
			sendQueue.notify();
		}
	}
	
	//called by the sending thread to retrieve the next data string to be sent after it has been instructed by receive thread to do so
	String getNextQueueItem() throws InterruptedException
	{
		synchronized (sendQueue)
		{
			while( sendQueue.isEmpty() )
			{
				sendQueue.wait();
			}
			
			return sendQueue.poll().data;
		}
	
	}
	
	void requestHistoryData(DateTime start, DateTime end, Vector<Integer> nodeNums, int intervalSeconds)
	{
		String str = start.toString() + ";" + end.toString() + ";";
		for(int j=0; j<nodeNums.size(); j++)
		{
			str = str + nodeNums.get(j);			
			if(j != nodeNums.size() -1)
			{
				str += ",";
			}
		}
		str = str + ";" + intervalSeconds;
		
		sendStringToServer(20, 6, str);
	}
	
	//String imageSignal = "";
	DateTimeFormatter dtf = DateTimeFormat.forPattern("HH:mm:ss : ");
	
	void processIncomingData(String incomingMessage)
	{
		 /*
		  * message format:
		  * 	1@data
		  * 
		 * send data types 
		 * 
		 * keep alive =1
		 * blackboard data update = 2
		 * blackboard header = 3
		 * debug data update = 4
		 * thread data header = 5
		 * select line info = 6
		 * log text message = 7
		 * 
		 */
		
		String[] parts = incomingMessage.split("@");
		
		//parts length must be 2
		if(parts.length != 2)
		{
			return;
		}
		
		int msgType = 0;
		try
		{
			msgType = Integer.parseInt(parts[0]);
		}catch ( NumberFormatException ex)
		{
			displayInLog(30, "corrupted message received");
		}
		
		String msg = parts[1];
		
		switch(msgType)
		{
			case 1:	//keep alive
			{
				//v got out keep alive back, do nothing, all is well
				updateLastKeepAlive();
				//displayInLog(1, "keep alive acknowledged");
				break;
			}
			case 2:	//blackboard data update
			{
				//System.out.println("Recived: " + msg);
				parentRobot.fillMemoryValuesFromString(parentRobot.blackboard, msg);
				parentJFrame.guiHelperThread.operationQueue.addDataToOperationQueue(new int[]{2,parentRobot.id}, null);
				break;
			}
			case 3:	//blackboard header update
			{
				parentRobot.fillMemoryNamesFromString(parentRobot.blackboard, msg);
				parentRobot.fillMemoryNamesFromString(parentRobot.debugMemory, msg);
				parentJFrame.guiHelperThread.operationQueue.addDataToOperationQueue(new int[]{1,parentRobot.id}, null);
				break;
			}
			case 4:	//debug data update
			{
				parentRobot.fillMemoryValuesFromString(parentRobot.debugMemory, msg);
				parentJFrame.guiHelperThread.operationQueue.addDataToOperationQueue(new int[]{3,parentRobot.id}, null);
				break;
			}
			case 5:	//threads connectors link header
			{
				// System.out.println(msg);
				parentRobot.createThreadsFromHeader(msg);
				parentJFrame.getRobotPanelFromRobotId(parentRobot.id).setThreadDataForThreadConnectorPanels();
				parentJFrame.guiHelperThread.operationQueue.addDataToOperationQueue(new int[]{4,parentRobot.id}, null);
				break;
			}
			case 6:	//thread connectors select line info
			{
				// System.out.println("Recived: " + msg);
				parentRobot.setThreadConnectorSelectLines(msg);
				break;
			}
			case 7:	//log text mesage
			{
				//msg.replace('\n', '\t');
				//System.out.println(msg);
				parentJFrame.getRobotPanelFromRobotId(parentRobot.id).commandControl.addToLog(5, dtf.print(DateTime.now()) +  msg);
				break;
			}
			case 8:	//update the image
			{
					//System.out.println("Image received");
					byte[] raw_data = Utilities.hexStringToByteArray(msg);	
					//System.out.println(Integer.toString(raw_data.length));
					Mat m = new Mat(1, raw_data.length, CvType.CV_8UC1);
					m.put(0, 0, raw_data);
					
					
					Mat mat = Highgui.imdecode(m, Highgui.CV_LOAD_IMAGE_COLOR);	
					
		            
					parentJFrame.getRobotPanelFromRobotId(parentRobot.id).updateCameraImage(mat);
//				}else
//				{
//					//append data
//					imageSignal += msg.substring(1);
//					System.out.println(imageSignal.length());
//				}

				break;
			}
			case 9:	//special msgs... footstep
			{
				byte[] raw_data = Utilities.hexStringToByteArray(msg);	
				//System.out.println(Integer.toString(raw_data.length));
				
				ByteBuffer bb = ByteBuffer.wrap(raw_data);
				bb.order(ByteOrder.LITTLE_ENDIAN);
				
				FloatBuffer floatBuffer = bb.asFloatBuffer();
				float[] result = new float[floatBuffer.remaining()];
				floatBuffer.get(result);
				System.out.println("got footsteps");
				System.out.println(Arrays.toString(result) );
				parentJFrame.getRobotPanelFromRobotId(parentRobot.id).robot.setFootStepsFromBytes(result);
				parentJFrame.fieldPanel.repaint();
				break;
			}

			//---------------------------------------------------------------------------------------------------------------------------
			/*
			 * localization
			 */
			//---------------------------------------------------------------------------------------------------------------------------
			//---------------------------------------------------------------------------------------------------------------------------
			
			case 10:	//special msgs... pf states
			{

				//System.out.println("PFStates received");
				byte[] raw_data = Utilities.hexStringToByteArray(msg);	
				//System.out.println(Integer.toString(raw_data.length));
				
				ByteBuffer bb = ByteBuffer.wrap(raw_data);
				bb.order(ByteOrder.LITTLE_ENDIAN);
				
				FloatBuffer floatBuffer = bb.asFloatBuffer();
				float[] result = new float[floatBuffer.remaining()];
				floatBuffer.get(result);
				
				//System.out.println("got PF states");
				//System.out.println(Arrays.toString(result) );
				
				parentJFrame.getRobotPanelFromRobotId(parentRobot.id).robot.setPFStateFromBytes(result);
				parentJFrame.fieldPanel.repaint();
				break;
			}
			default:	
			{
				
			}
			

		}
		
	}
	
	public void sendUserCommand(String command)
	{
		command = command.trim();
		sendStringToServer(20, 4, command);
	}
	
	public void sendCommand(String command)
	{
		sendStringToServer(20, 2, command);
	}
	
	public void sendSelectLine(String selectLine)
	{
		sendStringToServer(20, 3, selectLine);
	}

	
	
	/*
	 * send message types
	 * 
	 * keep alive message = 1
	 * force values = 2
	 * select line change = 3
	 * user command = 4;
	 */
	public void sendStringToServer(int priority, int type, String msg)
	{
		addDataToSendQueue(priority, type + "@" + msg);
	}
	
    //output data to the main log list from external classes	
    public void displayInLog(int priority, String str)
    {
    	logControl.addToLog(new LogObject(priority, parentRobot.name + "- " + str));
    }
	
    //stop the send and receiev threads and make this network handler inactive
    //check at the start if the network handler is currently active  or not
	//callingThread = 1 for recvThread & callingThread = 2 for sendThread  
	synchronized void cleanup(int callingThread)
	{
		displayInLog(1, "cleaning up socket");
		
		if(sendThread != null)
		{
			connected.set(false);
			try {
				
				//stop the other communication thread first
				if(callingThread == 1)	//if called by the recv thread
				{
					displayInLog(1, "stopping send thread");
					sendThread.stop();
				}
				else
				{
					displayInLog(1, "stopping recv thread");
					recvThread.stop();
				}
				
				socket.close();
				
				//remove
				//parentHandle.deleteServicingHandle(this);
				
				
				//stop the current thread now
				if(callingThread == 1)	//if called by the recv thread
				{
					displayInLog(1, "stopping recv thread");
					recvThread.stop();
				}
				else
				{
					displayInLog(1, "stopping send thread");
					sendThread.stop();
				}
				
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}


	public Socket getSocket() {
		return socket;
	}

	public String getServerName() {
		return serverName;
	}
	
	public int getSocketTimeout()
	{
		return socketTimeout;
	}

	public int getPort() {
		return port;
	}
	
	public void setConnectionSettings(String serverName, int port, int socketTimeout, int connRetryInterval)
	{
		this.serverName = serverName;
		this.port = port;
		this.socketTimeout = socketTimeout;	
		this.connRetryInterval = connRetryInterval;
	}
	
	private void updateLastKeepAlive()
	{
		lastKeepAlive.set( System.currentTimeMillis( ) );
	}
}


/**
 * ------------------------------------------------------------
 * **/
//The network receive class
//handles network data reception and performs the database operations
//finally it signals the send thread to send the response back
class RecvThread implements Runnable {
	
	//variables
	NetworkHandler parentHandle;
	BufferedReader in;
	
	//constructor
	RecvThread(NetworkHandler ParentHandle)
	{
		parentHandle = ParentHandle;
	}
	
	
	//the main function, this has the main thread loop
	public void run() {
		
		
		//set up the initial receipt variables and connections
		try{
		    in = new BufferedReader(new InputStreamReader(parentHandle.socket.getInputStream()));
		    
		    parentHandle.socket.setSoTimeout(parentHandle.getSocketTimeout());		    
		} catch (IOException e) {
			parentHandle.displayInLog(40, "in failed");
		    
		    //exit the thread
			parentHandle.cleanup(1);
		}
		
		parentHandle.displayInLog(1, "Recv thread started");
	
		//keep on receiving data and serciving the instruction
		while(true)
		{
			try{
				
				String incomingData = in.readLine();
				// System.out.println("Recived: " + incomingData);
				if(incomingData == null)
				{
					parentHandle.displayInLog(40, "Error while reading data");
				    
				  //exit the thread
				    parentHandle.cleanup(1);
				}
				
				serviceReceivedInstruction(incomingData);
				
			} catch (IOException e) {
				parentHandle.displayInLog(40, "Error reading data from socket");
			    
			    //exit the thread
				parentHandle.cleanup(1);
			}
		}
	}

	
	//service a received instruction
	//perform database operation if required
	//send signal to the send thread to send back the response to the client
	private void serviceReceivedInstruction(String incomingData)
	{
		//fetch some data from database and send to the sending thread
		//System.out.println("Recived: " + incomingData);
		
		//parentHandle.displayInLog(10, "Recived: " + incomingData); 
		parentHandle.processIncomingData(incomingData);
	}
}


/**
 * ------------------------------------------------------------
 * **/
//The network send class
//handles network data sending
//sendQueue temporarily holds the data which has to be sent
//gets notified by the receive thread that there is data pending to be sent in the send queue
class SendThread implements Runnable {
	
	//variables
	NetworkHandler parentHandle;
	PrintWriter out;
	
	//constructor
	SendThread(NetworkHandler ParentHandle)
	{
		parentHandle = ParentHandle;
	}
	
	
	//the main function, this has the main thread loop
	public void run() {
		
		//set up the initial sending variables and connections
		try{
			out = new PrintWriter(parentHandle.socket.getOutputStream(), true);
		} catch (IOException e) {
			parentHandle.displayInLog(40, "out failed");
			
		    //exit the thread if we cannot establish a sending connection
			parentHandle.cleanup(2);
		}
		
		parentHandle.displayInLog(1, "Send thread started");
		
		//get an item from the queue and send it to the client
		//getting and item from queue may involve waiting on a notification by the receive thread
		while(true)
		{
			try {
				
				String Data = parentHandle.getNextQueueItem();
				sendData(Data);		
			}
			catch(InterruptedException ex) {
				parentHandle.displayInLog(40, "Send Thread interrupted");
				parentHandle.displayInLog(40, ex.getMessage());
			} 
			catch (IOException e) 
			{
				parentHandle.displayInLog(40,"Error sending data to socket");
			    //exit the thread
				parentHandle.cleanup(2);
			} 
		}
	}

	
	//send data to the client
	private void sendData(String data) throws IOException
	{
		//parentHandle.displayInLog(1, "Sending: " + data);
		out.println(data);
	}
}
