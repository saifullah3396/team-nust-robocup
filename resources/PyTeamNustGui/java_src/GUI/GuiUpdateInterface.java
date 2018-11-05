package GUI;

public interface GuiUpdateInterface {
	void GuiUpdateComponent(Object data);
	int updateComponentId = -1;
}




//class LogListUpdate implements GuiUpdateInterface
//{
//	DefaultTableModel stationaryNodesTableModel;
//	DefaultTableModel mobileNodesTableModel;
//	
//	NodesListsUpdater( DefaultTableModel stationaryNodesTableModel, DefaultTableModel mobileNodesTableModel )
//	{
//		this.stationaryNodesTableModel = stationaryNodesTableModel;
//		this.mobileNodesTableModel = mobileNodesTableModel;
//	}
//	
//	@Override
//	public void GuiUpdateComponent(Object dataObject) 
//	{
//		// TODO Auto-generated method stub
//		DataBatch data = (DataBatch) dataObject;
//	
//	
//		//nodes list update operation
//		//the data was of   DataBatch   type
//		
//		//for(int j=0 ; j<stationaryNodesTableModel.getRowCount()-1; j++)
//		for(int j=0 ; j<stationaryNodesTableModel.getRowCount(); j++)
//		{
//			int currentNodeInternalId = (int) stationaryNodesTableModel.getValueAt(j, 0);
//			
//			DataSet currentDataSet = data.dataSets[currentNodeInternalId];
//			
//			for(int m=0; m<currentDataSet.dataFields.length; m++)
//			{
//				stationaryNodesTableModel.setValueAt(currentDataSet.dataFields[m], j, 3+m);
//			}
//			
//			//write the snapshot info to table columns
//			for(int k=0; k<currentDataSet.nodeInfo.snapshotFields.length; k++)
//			{
//				if(k == 0)
//				{
//					stationaryNodesTableModel.setValueAt(currentDataSet.nodeInfo.snapshotFields[k], j, k+3+currentDataSet.dataFields.length);
//			
//				}
//			}			
//		}
//		
//		for(int j=0 ; j<mobileNodesTableModel.getRowCount(); j++)
//		{
//			int currentNodeInternalId = (int) mobileNodesTableModel.getValueAt(j, 0);
//			
//			DataSet currentDataSet = data.dataSets[currentNodeInternalId];
//			
//			for(int m=0; m<currentDataSet.dataFields.length; m++)
//			{
//				mobileNodesTableModel.setValueAt(currentDataSet.dataFields[m], j, 3+m);
//			}
//			
//			//write the snapshot info to table columns
//			for(int k=0; k<currentDataSet.nodeInfo.snapshotFields.length; k++)
//			{
//				if(k == 0)
//				{
//					mobileNodesTableModel.setValueAt(currentDataSet.nodeInfo.snapshotFields[k], j, k+3+currentDataSet.dataFields.length);
//			
//				}
//			}
//		}
//			
//	}
//}
