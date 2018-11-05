package GUI;

import java.awt.BorderLayout;
import java.awt.event.ActionEvent;
import java.awt.event.MouseEvent;
import java.util.Vector;

import javax.swing.AbstractAction;
import javax.swing.Action;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTable;
import javax.swing.table.DefaultTableModel;

import net.miginfocom.swing.MigLayout;
import DataContainers.Memory;
import DataContainers.ThreadData;
import DataContainers.Variable;

public class ThreadConnectionsPanel extends JPanel {

	RobotPanel parentPanel; 
	private JTable inputTable;
	
	private JTable outputTable;
	
	DefaultTableModel inputTableModel;
	DefaultTableModel outputTableModel = new DefaultTableModel();
	
	ThreadData threadData;
	
	/**
	 * Create the panel.
	 */
	public ThreadConnectionsPanel(final RobotPanel parentPanel) {
		this.parentPanel = parentPanel;
		setLayout(new BorderLayout(0, 0));
		JPanel panel = new JPanel();
		panel.setLayout(new MigLayout("", "[202.00px,grow][119px,grow 60]", "[][234px,grow]"));
		this.add(panel);
		JLabel lblInputs = new JLabel("Inputs");
		panel.add(lblInputs, "cell 0 0,alignx center");
		
		JLabel lblOutputs = new JLabel("Outputs");
		panel.add(lblOutputs, "cell 1 0,alignx center");
		
		JScrollPane scrollPane_2 = new JScrollPane();
		panel.add(scrollPane_2, "cell 0 1,grow");
		
		inputTable = new JTable(){

            //Implement table cell tool tips.           
            public String getToolTipText(MouseEvent e) {
                String tip = null;
                java.awt.Point p = e.getPoint();
                int rowIndex = rowAtPoint(p);
                int colIndex = columnAtPoint(p);

                try {
                    //comment row, exclude heading
                    if(rowIndex != 0){
                      tip = getValueAt(rowIndex, colIndex).toString();
                    }
                } catch (RuntimeException e1) {
                    //catch null pointer exception if mouse is over an empty line
                }

                return tip;
            }
        };	
        
		inputTableModel = new DefaultTableModel(
			new Object[][] {
			},
			new String[] {
				"#", "Name" ,"Memory", "Force"
			}
		);
		inputTable.setModel(inputTableModel);
		scrollPane_2.setViewportView(inputTable);
		
		JScrollPane scrollPane_3 = new JScrollPane();
		panel.add(scrollPane_3, "cell 1 1,grow");
		
		outputTable = new JTable(){

            //Implement table cell tool tips.           
            public String getToolTipText(MouseEvent e) {
                String tip = null;
                java.awt.Point p = e.getPoint();
                int rowIndex = rowAtPoint(p);
                int colIndex = columnAtPoint(p);

                try {
                    //comment row, exclude heading
                    if(rowIndex != 0){
                      tip = getValueAt(rowIndex, colIndex).toString();
                    }
                } catch (RuntimeException e1) {
                    //catch null pointer exception if mouse is over an empty line
                }

                return tip;
            }
        };
        
		outputTableModel = new DefaultTableModel(
			new Object[][] {
			},
			new String[] {
				"#", "Name", "Value"
			}
		);
		outputTable.setModel(outputTableModel);
		scrollPane_3.setViewportView(outputTable);

		inputTable.getColumnModel().getColumn(2).setCellRenderer(new CustomCellRenderer());	
		inputTable.getColumnModel().getColumn(3).setCellRenderer(new CustomCellRenderer());	
				
		Action inputTableAction = new AbstractAction()
		{
		    public void actionPerformed(ActionEvent e)
		    {
		    	
		        TableCellListener tcl = (TableCellListener)e.getSource();
		        
		        if(tcl.getColumn() == 3)	//if debug force column
		        {
			        String val = (String)tcl.getNewValue();
			        String old = (String)tcl.getOldValue();
			        
			        
			        if(!val.equals(old))
			        {
			        	if(val.startsWith("*"))
		                {
		                	val = val.substring(1);
		                }
			        	
			        	String command = Integer.toString( threadData.threadid ) + "," + Integer.toString(tcl.getRow() ) + "," + val;
			        	parentPanel.sendCommandToRobot(command);
			        	inputTableModel.setValueAt("*" + val, tcl.getRow(), tcl.getColumn());
			        } 
			        
			        
		        }else if(tcl.getColumn() == 2)
		        {
		        	String val = (String)tcl.getNewValue();
			        String old = (String)tcl.getOldValue();
			        
			        
			        if(!val.equals(old))
			        {
			        	inputTableModel.setValueAt(tcl.getOldValue(), tcl.getRow(), tcl.getColumn());
			        	String command = Integer.toString( threadData.threadid ) + "," + Integer.toString(tcl.getRow() ) + ",0";
			        	parentPanel.sendSelectLine(command);
			        }
		        	
		        }else
		        {
		        	inputTableModel.setValueAt(tcl.getOldValue(), tcl.getRow(), tcl.getColumn());
		        }
		    }
		};
		
		TableCellListener tcl = new TableCellListener(inputTable, inputTableAction);
	}
	
	

	
	public void setThreadData(ThreadData threadData)
	{
		this.threadData = threadData;
	}
	
	void updateHeaderNames()
	{
		Memory blackboard = parentPanel.robot.blackboard;
		//delete all rows
		int rows = inputTableModel.getRowCount(); 
		for(int i = rows - 1; i >=0; i--)
		{
			inputTableModel.removeRow(i); 
		}
		
		rows = outputTableModel.getRowCount(); 
		for(int i = rows - 1; i >=0; i--)
		{
			outputTableModel.removeRow(i); 
		}
		
		for(int j=0; j<threadData.inputConnector.variables.size(); j++)
		{
			Variable currentVariable = threadData.inputConnector.variables.get(j);
			Vector<String> rowData = new Vector<String>();
			rowData.add(Integer.toString( j));
			rowData.add(currentVariable.name);
			rowData.add("");
			rowData.add("");
			inputTableModel.addRow(rowData);
		}
		
		for(int j=0; j<threadData.outputConnector.variables.size(); j++)
		{
			Variable currentVariable = threadData.outputConnector.variables.get(j);
			Vector<String> rowData = new Vector<String>();
			rowData.add(Integer.toString( j));
			rowData.add(currentVariable.name);
			rowData.add("");
			outputTableModel.addRow(rowData);
		}
			
	}
	
	void updateBlackboardMemoryValues()
	{
		if(inputTableModel.getRowCount() != 0)
		{
			for(int j=0; j<threadData.inputConnector.variables.size(); j++)
			{
				if(threadData.inputConnector.selectLines.get(j) == 0)
				{
					inputTableModel.setValueAt('*' + threadData.inputConnector.variables.get(j).value, j, 2);
					
				}else
				{
					inputTableModel.setValueAt( threadData.inputConnector.variables.get(j).value, j, 2);
				}
			}
		}
		
		if(outputTableModel.getRowCount() != 0)
		{
			for(int j=0; j<threadData.outputConnector.variables.size(); j++)
			{
				outputTableModel.setValueAt( threadData.outputConnector.variables.get(j).value, j, 2);
			}
		}
	}
	
	void updateDebugMemoryValues()
	{
		if(inputTableModel.getRowCount() != 0)
		{
			for(int j=0; j<threadData.inputConnector.variables.size(); j++)
			{
				if(threadData.inputConnector.selectLines.get(j) == 1)
				{
					inputTableModel.setValueAt( '*' + threadData.inputConnector.forceVariable.get(j).value, j, 3);
				}else
				{
					inputTableModel.setValueAt( threadData.inputConnector.forceVariable.get(j).value, j, 3);
				}
			}
		}

	}
	
	
	public JTable getInputTable() {
		return inputTable;
	}
	public JTable getOutputTable() {
		return outputTable;
	}
}


