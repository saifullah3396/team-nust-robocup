package LogListController;

import java.awt.Color;
import java.awt.Component;
import java.awt.Font;
import java.util.LinkedList;

import javax.swing.DefaultListModel;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.ListCellRenderer;


public class LogLevelHandler {
	JList<LogObject> logList;
	DefaultListModel<LogObject> logListModel; 
	JFrame parentJFrame;
	exiter appExitInterface;
	
	public boolean autoScroll = true;
	public boolean verbose = true;
	public boolean information = true;
	public boolean errors = true;
	public boolean pause = false;
	public boolean beep = false;
	
	public boolean growAtHead = false;
	
	public LinkedList<LogObject> logQueue = new LinkedList<LogObject>();
	
	public LogLevelHandler(JList<LogObject> LogList, DefaultListModel<LogObject> LogListModel, exiter AppExitInterface )
	{
		appExitInterface = AppExitInterface;
		logList = LogList;
		logListModel = LogListModel; 
		logList.setCellRenderer(new LogListRender());
	}
	
	
	public void addToLog(int level, String message)
	{
		addToLog(new LogObject(level, message));
	}
	
	public void addToLog(LogObject logData)
	{
		synchronized(logQueue)
		{
			logQueue.addLast(logData);
		}
	}
	
	public void printLogQueue()
	{
		synchronized(logQueue)
		{
			if(!pause)
			{
				while(true)
				{
					LogObject log = logQueue.pollFirst();
					if(log != null)
					{
						print(log);
					}else
					{
						break;
					}
				}
			}
		}
	}
	
	private void print(LogObject logData)
	{
    	//decide what this list object is about to show
    	if(logData.priorityLevel < 10)	//if a verbose message
		{
			if(verbose)
			{
				//System.out.println(logData.data);
				printToList(logData);   
			}
			
		}else if(logData.priorityLevel < 20)	//if a normal message
		{
			if(information)
			{
				//System.out.println(logData.data);
				printToList(logData);   
			}
		}else	//if an error message
		{	
			if(errors)
			{
				//System.out.println(logData.data);
				printToList(logData);   
				if(beep)
				{
					java.awt.Toolkit.getDefaultToolkit().beep();
				}
			}
		}
	}

	private void printToList(LogObject logData)
	{
		if(!growAtHead)
		{
			logListModel.addElement(logData);
	        
	        if(autoScroll){
	        	logList.setSelectedIndex(logListModel.size() -1);
	        	logList.ensureIndexIsVisible( logList.getSelectedIndex() );
	        }
		}else
		{
			logListModel.add(0,  logData);
		}
	}
	
	
	class LogListRender extends JLabel implements ListCellRenderer<LogObject> {

	    public LogListRender() {
	        setOpaque(true);
	    }

	    public Component getListCellRendererComponent(JList list, LogObject value, int index, boolean isSelected, boolean cellHasFocus) {
	           	
	    	//set the colours
	    	if(value.priorityLevel < 10)	//if a verbose message
			{
	    		 setForeground(Color.GRAY);
				
			}else if(value.priorityLevel < 20)	//if a normal message
			{
				 setForeground(Color.BLACK);
			}else	//if an error message
			{	
				 setForeground(Color.RED);
			}
	    	
	    	setFont( new Font("tahoma", Font.PLAIN, 12));
	    	
	    	// Set the data string as the list row text
	        setText(value.data.toString());
	        

	        // based on the index you set the color.  This produces the every other effect.
	        if (index % 2 != 0) 
	        {
	        	setBackground(Color.decode("0XFAFAFA"));
	        }else
	        {
	        	setBackground(Color.white);
	        }

	        return this;
	    }
	}

	
	public void showInDialog(int messageId, String extraInfo, boolean exitApplication)
	{
		String message = null;
		String title = null;
		int dialogType = 0;
		
		switch(messageId)
		{
			case 1000:
			{
				message = "Unable to open socket."
					      	+ "\nAnother instance of this application is already running OR system is not configured properly.\n"
					      	+ "Restart your computer and try again.";
				title = "Initiailisation Error";
				dialogType = JOptionPane.ERROR_MESSAGE;

				break;
			}
			case 1001:
			{
				message = "Unable to update settings to database."
					    + "\nRestart your computer and try again.\n";
					  
			title =   "Database Update Error";
			dialogType = JOptionPane.ERROR_MESSAGE;
				break;
			}
			
			case 1002:
			{
				message = "Unable to connect to Database to fetch system information."
					    + "\nRecheck Database connection settings and try again.\n";
					  
			title =   "Database Connection Error";
			dialogType = JOptionPane.ERROR_MESSAGE;
				break;
			}
			
			case 1003:
			{
				message = "Unable to connect to fetch nodes information from database."
					    + "\nDatabase might be corrupted.\n";
					  
			title =   "Database Connection Error";
			dialogType = JOptionPane.ERROR_MESSAGE;
				break;
			}
			
			case 1004:
			{
				message = "Unable to connect to fetch data field information from database."
					    + "\nDatabase might be corrupted.\n";
					  
			title =   "Database Connection Error";
			dialogType = JOptionPane.ERROR_MESSAGE;
				break;
			}
			
			
			
			//client messages handler----------------------------------------
			case 2001:
			{
				message = "No Data Series added to list."
					    + "\nAdd at least one Data Series and try again.\n";
					  
			title =   "Graph Fetch Error";
			dialogType = JOptionPane.ERROR_MESSAGE;
				break;
			}
			
			case 2002:
			{
				message = "Graph start time is after the end time."
					    + "\nSelect a valid time window and try again.\n";
					  
			title =   "Graph Fetch Error";
			dialogType = JOptionPane.ERROR_MESSAGE;
				break;
			}
		}

		JOptionPane.showMessageDialog(parentJFrame,
			    message,
			    title,
			    dialogType);
		
		if (exitApplication)
		{
			//parentJFrame.exitApplication();
			appExitInterface.exitApplication();
		}
	}

}

