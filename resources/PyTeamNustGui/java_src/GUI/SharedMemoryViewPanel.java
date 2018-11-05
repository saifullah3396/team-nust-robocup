package GUI;

import java.awt.BorderLayout;
import java.util.Vector;

import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

import DataContainers.Memory;
import DataContainers.Variable;

public class SharedMemoryViewPanel extends JPanel {
	private JTable table;
	private DefaultTableModel tableModel;
	
	Memory blackboardMemory;
	Memory debugMemory;
	/**
	 * Create the panel.
	 */
	
	
	public SharedMemoryViewPanel() {
		setLayout(new BorderLayout(0, 0));
		JScrollPane scrollPane = new JScrollPane();
		add(scrollPane);
		
		table = new JTable();
		tableModel = new DefaultTableModel(
				new Object[][] {
				},
				new String[] {
					"Index", "Name", "BlackBoard", "Debug"
				}
			);
		table.setModel(tableModel);
				
		scrollPane.setViewportView(table);
	}
	
	public void updateDebugMemoryValues()
	{
		for(int j=0; j<debugMemory.variables.size(); j++)
		{
			tableModel.setValueAt( debugMemory.variables.get(j).value, j, 3);
		}
	}
	
	public void updateBlackboardMemoryValues()
	{
		if(tableModel.getRowCount() != 0)
		{
			for(int j=0; j<blackboardMemory.variables.size(); j++)
			{
				tableModel.setValueAt( blackboardMemory.variables.get(j).value, j, 2);
			}
		}
	}

	public void updateMemoryNames()
	{
		//delete all rows
		int rows = tableModel.getRowCount(); 
		for(int i = rows - 1; i >=0; i--)
		{
			tableModel.removeRow(i); 
		}
	
		
		for(int j=0; j<blackboardMemory.variables.size(); j++)
		{
			Variable currentVariable = blackboardMemory.variables.get(j);
			Vector<String> rowData = new Vector<String>();
			rowData.add(Integer.toString( currentVariable.index));
			rowData.add(currentVariable.name);
			rowData.add("");
			tableModel.addRow(rowData);
		}
	}
	
	void setBlackboardMemory(Memory blackboardMemory)
	{
		this.blackboardMemory = blackboardMemory;
	}
	
	void setDebugMemory(Memory debugMemory)
	{
		this.debugMemory = debugMemory;
	}
	
	public JTable getTable() {
		return table;
	}
}
