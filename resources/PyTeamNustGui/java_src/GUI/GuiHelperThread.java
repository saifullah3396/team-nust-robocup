package GUI;
import java.util.LinkedList;
import java.util.List;
import java.util.Vector;

import javax.swing.SwingWorker;

import LogListController.LogLevelHandler;



public class GuiHelperThread extends SwingWorker<String, String> {
	
	int updateInterval;
	int graphHistoryUpdateTimeout;
	
	Vector<LogLevelHandler> logControls = new Vector<LogLevelHandler> ();
	MainFrame parentJFrame;
	
	public OperationQueue operationQueue = new OperationQueue();
	
	Vector<GuiUpdateInterface> realtimeUpdateComponents = new Vector<GuiUpdateInterface>();
	Vector<GuiUpdateInterface> historyUpdateComponents = new Vector<GuiUpdateInterface>();
	

	GuiHelperThread(MainFrame ParentJFrame, LogLevelHandler LogControl)
	{
		parentJFrame = ParentJFrame;
		logControls.add(  LogControl );
		updateInterval = 500;
	}
	
	@Override
	protected String doInBackground() throws Exception {
		// TODO Auto-generated method stub
		
		while(true)
		{
			try {
				
				

	
				publish("hello");

    		    Thread.sleep(updateInterval);
    		} catch(InterruptedException ex) {
    		    Thread.currentThread().interrupt();
    		}
		}
	}
	  
    @Override
    protected void process(List<String> chunks) {
	//         for (int j=0; j<chunks.size(); j++) {
	//        
	//         }
    	//do the periodic GUI update here
    	executeOperationQueue();
    	
    	for(int j=0; j<logControls.size(); j++)
    	{
    		logControls.get(j).printLogQueue();
    	}
    }
        


	
	public void executeOperationQueue()
	{
		synchronized(operationQueue)
		{
			while(true)
			{
				UpdateOperation operation = operationQueue.theQueue.pollFirst();
				if(operation != null)
				{
					//process the opeartion here
					
					switch(operation.type.get(0))
					{
						case 1:	 // 1: robot panel: shared memory: blackboard: header 	//idVariable = robotId
						{		
							parentJFrame.getRobotPanelFromRobotId(operation.type.get(1)).getBlackboardPanel().updateMemoryNames();
							break;
						}
						case 2: // 2: robot panel: shared memory: blackboard: values 	//idVariable = robotId
						{
							parentJFrame.getRobotPanelFromRobotId(operation.type.get(1)).getBlackboardPanel().updateBlackboardMemoryValues();
							parentJFrame.getRobotPanelFromRobotId(operation.type.get(1)).updateBlackboardMemoryValues();
							break;
						}
						case 3: // 2: robot panel: shared memory: debug force: values 	//idVariable = robotId
						{
							parentJFrame.getRobotPanelFromRobotId(operation.type.get(1)).getBlackboardPanel().updateDebugMemoryValues();
							parentJFrame.getRobotPanelFromRobotId(operation.type.get(1)).updateDebugMemoryValues();
							break;
						}
						case 4: // 2: robot panel: connectors: threadid : input/output: values 	//idVariable = robotId
						{
							//update all thread connector tables
							parentJFrame.getRobotPanelFromRobotId(operation.type.get(1)).updateThreadConnectorTableHeaders();
							break;
						}
					
						
//						case 3:	//nodes record update
//						{
//							//null is received as data
//							parentJFrame.fillNodesListRows();
//							parentJFrame.fillNodesDropDown();
//							
//							parentJFrame.intialConnectionEstablished();
//							break;
//						}
//						
//						case 4:	//history data update
//						{
//							if(operation.data != null)
//								updateHistoryComponents(operation.data);
//							}
//							updateHistoryGraphProgress();
//							break;
//						}
					}

				}else
				{
					break;
				}
			}
		}
	}
		
	
	//realtime components --------------------------------------
	
	void addToRealTimeComponents(GuiUpdateInterface updateComponentInterface)
	{
		synchronized(realtimeUpdateComponents)
		{
			if(!realtimeUpdateComponents.contains(updateComponentInterface))
				realtimeUpdateComponents.add(updateComponentInterface);
		}
	}
	
	void deleteFromRealTimeComponents(GuiUpdateInterface updateComponentInterface)
	{
		synchronized(realtimeUpdateComponents)
		{
			realtimeUpdateComponents.remove(updateComponentInterface);
		}
	}
	
	void updateRealtimeComponents(Object objectData)
	{
		synchronized(realtimeUpdateComponents)
		{
			for(int j=0; j<realtimeUpdateComponents.size(); j++)
			{
				realtimeUpdateComponents.get(j).GuiUpdateComponent(objectData);
			}
		}
	}	
	
	
	//history components --------------------------------------
	
		void addToHistoryComponents(GuiUpdateInterface updateComponentInterface)
		{
			synchronized(historyUpdateComponents)
			{
				if(!historyUpdateComponents.contains(updateComponentInterface))
					historyUpdateComponents.add(updateComponentInterface);
			}
		}
		
		void deleteFromHistoryComponents(GuiUpdateInterface updateComponentInterface)
		{
			synchronized(historyUpdateComponents)
			{
				historyUpdateComponents.remove(updateComponentInterface);
			}
		}
		
		void updateHistoryComponents(Object objectData)
		{
			synchronized(historyUpdateComponents)
			{
				for(int j=0; j<historyUpdateComponents.size(); j++)
				{
					historyUpdateComponents.get(j).GuiUpdateComponent(objectData);
				}
			}
		}	
}

	


/*
 * 
 * update operation types
 * 
 * 1: robot panel: shared memory: blackboard: header 	//idVariable = robotId
 * 2: robot panel: shared memory: blackboard: values 	//idVariable = robotId
 */

//Update Operation Queue data and classes//
/*** Update Operation Task ***/    
class UpdateOperation {
  public Vector<Integer> type = new Vector<Integer>();
  public Object data;
  
  UpdateOperation(int type[], Object data)
  {
	  for(int j=0; j<type.length; j++)
	  {
		  this.type.add(type[j]);
	  }
	 
	  this.data = data;
  }
}
//- Queue stuff ends here---------------------//


class OperationQueue
{
	public LinkedList<UpdateOperation> theQueue = new LinkedList<UpdateOperation>();
	
	//signal the update thread to do something
	void addDataToOperationQueue(int type[], Object data)
	{
		synchronized (theQueue)
		{
			theQueue.add(new UpdateOperation(type, data));
			theQueue.notify();
		}
	}
	
	//called by the gui helper thread to retrieve the next update operation which has to be performed
	Object getNextQueueItem() throws InterruptedException
	{
		synchronized (theQueue)
		{
			while( theQueue.isEmpty() )
			{
				theQueue.wait();
			}
			
			return theQueue.poll().data;
		}
	
	}
}