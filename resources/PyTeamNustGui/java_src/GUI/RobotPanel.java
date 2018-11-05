package GUI;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.util.Vector;

import javax.swing.DefaultListModel;
import javax.swing.ImageIcon;
import javax.swing.JCheckBox;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.JTextField;
import javax.swing.border.LineBorder;

import net.miginfocom.swing.MigLayout;
import DataContainers.Robot;
import LogListController.LogLevelHandler;
import LogListController.LogObject;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Image;

import javax.swing.border.TitledBorder;
import javax.swing.JLabel;

import org.opencv.core.Mat;
import java.awt.event.ActionListener;
import java.awt.event.ActionEvent;
import javax.swing.ListSelectionModel;


public class RobotPanel extends JPanel implements GuiUpdateInterface {
	//gui stuff
	MainFrame parentFrame;
	ImageIcon icon = new ImageIcon(); 
	
	//robot 
	Robot robot;
	
	//command panel stuff
	private JTextField commandInput;
	JList<LogObject> commandHistory = new JList<LogObject>();
	DefaultListModel<LogObject> commandHistoryModel = new DefaultListModel<LogObject>();
	public LogLevelHandler commandControl;
	
	//shared memory view
	private SharedMemoryViewPanel memoryPanel;
	
	//thread connectors view
	Vector<ThreadConnectionsPanel> threadPanels = new Vector<ThreadConnectionsPanel>();
	private JLabel imageLabel;


	
	/**
	 * Create the panel.
	 */
	public RobotPanel(MainFrame parentFrame, final Robot robot) 
	{
		this.parentFrame= parentFrame;
		this.robot = robot;
		setLayout(new MigLayout("", "[320px:n:320px,grow][6px][456px,grow]", "[239px,grow 80][240:n:240px,grow][130px,grow]"));
		
		JPanel cameraPanel = new JPanel();
		//cameraPanel.setBorder(null);
		add(cameraPanel, "cell 0 1,grow");
		cameraPanel.setLayout(new BorderLayout());
		
		imageLabel = new JLabel("");
		imageLabel.setBounds(-160, -120, 160, 120);
		imageLabel.setBorder(new LineBorder(Color.BLACK));
		cameraPanel.add(imageLabel, BorderLayout.CENTER);
        
		JPanel commandPanel = new JPanel();
		add(commandPanel, "cell 0 2 3 1,grow");
		
		commandInput = new JTextField();
		commandInput.addKeyListener(new KeyAdapter() {
			@Override
			public void keyTyped(KeyEvent arg0) {
				if(arg0.getKeyChar() == arg0.VK_ENTER)
				{
					//if enter is pressed
					String command = commandInput.getText();
					sendUserCommandToRobot(command);
					commandInput.setText("");
				}
			}
		});
		commandPanel.setLayout(new MigLayout("", "[496.00px,grow][150px:n]", "[110px,grow][20px]"));
		
		JScrollPane scrollPane_1 = new JScrollPane();
		commandPanel.add(scrollPane_1, "cell 1 0 1 2,grow");
		
		JList<JCheckBox> list = new JList<JCheckBox>();
		scrollPane_1.setViewportView(list);
		
		final JCheckBox chckbxAutoscroll = new JCheckBox("Autoscroll");
		chckbxAutoscroll.setSelected(true);
		chckbxAutoscroll.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
				
				commandControl.autoScroll = chckbxAutoscroll.isSelected();

			}
		});
		scrollPane_1.setColumnHeaderView(chckbxAutoscroll);
//		list.setModel(new AbstractListModel<JCheckBox>() {
//			JCheckBox[] values = new JCheckBox[] {new JCheckBox("Decision Thread"), new JCheckBox("Debug Thread")};
//			
//			public int getSize() {
//				return values.length;
//			}
//			public JCheckBox getElementAt(int index) {
//				return values[index];
//			}
//		});
		commandPanel.add(commandInput, "cell 0 1,growx,aligny top");
		commandInput.setText("0,0,250");
		commandInput.setColumns(10);
		
		JScrollPane scrollPane = new JScrollPane();
		commandPanel.add(scrollPane, "cell 0 0,grow");
		commandHistory.setSelectionMode(ListSelectionModel.SINGLE_INTERVAL_SELECTION);
		

		commandHistory.setModel(commandHistoryModel);
		scrollPane.setViewportView(commandHistory);
		commandControl = new LogLevelHandler(commandHistory, commandHistoryModel, parentFrame);
		
		
		JTabbedPane memoryTabbedPane = new JTabbedPane(JTabbedPane.TOP);
		add(memoryTabbedPane, "cell 0 0,grow");
		
		memoryPanel = new SharedMemoryViewPanel();
		memoryPanel.setBlackboardMemory(robot.blackboard);
		memoryPanel.setDebugMemory(robot.debugMemory);
		memoryTabbedPane.addTab("Memory", null, memoryPanel, null);

		JTabbedPane connectorTabbedPane = new JTabbedPane(JTabbedPane.LEFT);
		add(connectorTabbedPane, "cell 2 0 1 2,grow");
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Control Thread", null, threadPanels.lastElement(), null);
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Camera Thread", null, threadPanels.lastElement(), null);
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Comm Thread", null, threadPanels.lastElement(), null);	
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Processing Thread", null, threadPanels.lastElement(), null);
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Planning Thread", null, threadPanels.lastElement(), null);

		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Motion Thread", null, threadPanels.lastElement(), null);
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Static Behaviors Thread", null, threadPanels.lastElement(), null);
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Vision Thread", null, threadPanels.lastElement(), null);
		
		threadPanels.add(new ThreadConnectionsPanel(this));
		connectorTabbedPane.addTab("Localization Thread", null, threadPanels.lastElement(), null);
	}
	
	//---------------------------------------------------------------------------
	//camera functions
	
	void updateCameraImage(Mat img)
	{
		 Image image = toBufferedImage(img);
		 Image dimg = image.getScaledInstance(320, 240, Image.SCALE_DEFAULT);
		 icon.setImage(dimg);  
		 imageLabel.setIcon(icon);
		 imageLabel.repaint();
	}
	
	public Image toBufferedImage(Mat m)
	{
	      int type = BufferedImage.TYPE_BYTE_GRAY;
	      if ( m.channels() > 1 ) {
	          type = BufferedImage.TYPE_3BYTE_BGR;
	      }
	      int bufferSize = m.channels()*m.cols()*m.rows();
	      byte [] b = new byte[bufferSize];
	      m.get(0,0,b); // get all the pixels
	      BufferedImage image = new BufferedImage(m.cols(),m.rows(), type);
	      final byte[] targetPixels = ((DataBufferByte) image.getRaster().getDataBuffer()).getData();
	      System.arraycopy(b, 0, targetPixels, 0, b.length);  
	      return image;

	  }

	//--------------------------------------------------------------------------
	//robots send data interface
	void sendUserCommandToRobot(String command)
	{
		commandControl.addToLog(15, ">> UserCommand:" + command);
		commandHistory.ensureIndexIsVisible(commandHistoryModel.getSize()-1);
		robot.sendUserCommand(command);
	}
	void sendCommandToRobot(String command)
	{
		commandControl.addToLog( 15, ">> Command:" + command);
		commandHistory.ensureIndexIsVisible(commandHistoryModel.getSize()-1);
		robot.sendCommand(command);
	}
	
	public void sendSelectLine(String selectLine)
	{
		commandControl.addToLog( 15, ">> SelectLine:" + selectLine);
		commandHistory.ensureIndexIsVisible(commandHistoryModel.getSize()-1);
		robot.sendSelectLine(selectLine);
	}
	
	void updateThreadConnectorTableHeaders()
	{
		for(int j=0; j<threadPanels.size(); j++)
		{
			threadPanels.get(j).updateHeaderNames();
		}
	}
	
	
	//send update command to thread connector panels----------------------------------
	void updateBlackboardMemoryValues()
	{
		for(int j=0; j<threadPanels.size(); j++)
		{
			threadPanels.get(j).updateBlackboardMemoryValues();
		}
	}
	
	void updateDebugMemoryValues()
	{
		for(int j=0; j<threadPanels.size(); j++)
		{
			threadPanels.get(j).updateDebugMemoryValues();
		}
	}
		
	//link robot thread data vector to the dialog table panels
	void setThreadDataForThreadConnectorPanels()
	{
		for(int j=0; j<threadPanels.size(); j++)
		{
			threadPanels.get(j).setThreadData(robot.threadsData.get(j));
		}
	}
	
	public JTextField getCommandArea() {
		return commandInput;
	}
	public SharedMemoryViewPanel getBlackboardPanel() {
		return memoryPanel;
	}

	//public void updateSharedMemory 

	//for GUi update
	@Override
	public void GuiUpdateComponent(Object data) {
		// TODO Auto-generated method stub
		
	}
	public JLabel getImageLabel() {
		return imageLabel;
	}
}
