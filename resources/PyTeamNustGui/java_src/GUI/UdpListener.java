package GUI;

import java.awt.geom.Point2D;
import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Vector;

import org.joda.time.DateTime;
import org.joda.time.format.DateTimeFormat;
import org.joda.time.format.DateTimeFormatter;
import org.opencv.core.Point3;

public class UdpListener extends Thread 
{
	MainFrame parentFrame;
	DatagramSocket socket;
	
	int port;
	boolean connected = false;
	
	UdpListener(MainFrame parentFrame)
	{
		this.parentFrame = parentFrame;
		
	}
	
	public void connect() throws IOException
	{
//		socket = new MulticastSocket(port);
//	
//		InetAddress group = InetAddress.getByName(groupName);s
//		socket.joinGroup(group);
		//Keep a socket open to listen to all the UDP trafic that is destined for this port
		socket = new DatagramSocket(20000, InetAddress.getByName("0.0.0.0"));
		//socket.setBroadcast(true);

		connected = true;
	}
	
	void setSetting(int port)
	{
		this.port = port;
	}
	
	DateTimeFormatter dtf = DateTimeFormat.forPattern("HH:mm:ss : ");
	
	public void run()
	{
		
		DatagramPacket packet;
		while(true)
		{
			try 
			{
				if(!connected)
				{
					connect();
				}
				
				//***************************************************************************
				//max packet length = 10000
			    byte[] buf = new byte[10000];
			    packet = new DatagramPacket(buf, buf.length);
				socket.receive(packet);
				
				processIncommingData(packet);
				
			    
		    
			} catch (IOException e) {
				// TODO Auto-generated catch block
				//e.printStackTrace();
				parentFrame.logControl.addToLog(30, "Unable to bind to UDP Port");
			}

		}
			
	}
	
	//******************************************************************************
	//comm variables
	byte escapeVal = -1;
	byte packetLength = 28;
	
	byte[] decodePacket(DatagramPacket packet)
	{
		byte arr[] = packet.getData();
		
//		for(int j=0; j<arr.length; j++)
//		{
//			System.out.print( arr[j] + " " );
//		}
//		System.out.println();
		
		Vector<Byte> out = new Vector<Byte>();
		
		boolean escapeSeq = false;
		for(int j=0; j<arr.length; j++)
		{
			if(escapeSeq)
			{
				if( arr[j] == escapeVal)
				{
					out.add(arr[j]);
					escapeSeq = false;
				}else
				{
					//end of packet
					break;
				}
			}else
			{
				if( arr[j] == escapeVal)
				{
					escapeSeq = true;
				}else
				{
					out.add(arr[j]);
				}
			}
		
		}
		
		//System.out.println("got =" + out.toString());
		//make a byte array
		byte outArr[] = new byte[out.size()];
		for(int j=0; j<out.size(); j++)
		{
			outArr[j] = out.get(j);
		}
		
		return outArr;
		
	}
	
	void processIncommingData(DatagramPacket packet)
	{
		//String received = new String(packet.getData());
		//  create a byte buffer and wrap the array
		
		byte data[] = decodePacket ( packet ) ; 
		
		//System.out.println(Integer.toString( data.length) );
		if(data.length == packetLength)
		{
			
			
			ByteBuffer bb = ByteBuffer.wrap(data );
			
			bb.order(ByteOrder.LITTLE_ENDIAN);
			
			int robotId = bb.getInt();
			Point2D.Double ballLoc = new Point2D.Double(bb.getFloat(), bb.getFloat());
			Double ballDist = (double) bb.getFloat();
			Point3 RobotLoc = new Point3(bb.getFloat(), bb.getFloat(), bb.getFloat());
			
		    String received = "R=" + Integer.toString(robotId) + "; BallDist= " + ballDist + "; BallLoc= " + ballLoc +  "; RobotPose= " + RobotLoc + "***Raw=[";
		    for(int j=0; j<data.length; j++)
		    {
		    	received += data[j];
		    	if(j != data.length-1)
		    	{
		    		received += ",";
		    	}else
		    	{
		    		received += "]";
		    	}
		    }
	
		    received = dtf.print( DateTime.now() ) + received;
		    
		    parentFrame.udpLogControl.addToLog(10, received);
		   
		}
	    
	}
}
