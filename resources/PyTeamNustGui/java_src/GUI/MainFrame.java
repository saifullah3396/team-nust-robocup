package GUI;

import java.awt.BorderLayout;
import java.awt.EventQueue;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Vector;
import java.lang.System.*;

import javax.swing.AbstractButton;
import javax.swing.DefaultListModel;
import javax.swing.JCheckBoxMenuItem;
import javax.swing.JFrame;
import javax.swing.JList;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTabbedPane;
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.border.EmptyBorder;

import DataContainers.Field;
import LogListController.LogLevelHandler;
import LogListController.LogObject;
import LogListController.exiter;

public class MainFrame extends JFrame implements exiter {

	private JPanel contentPane;
	JTabbedPane tabbedPane;

	// gui stuff
	public GuiHelperThread guiHelperThread;
	public FieldPanel fieldPanel;

	// log varibales
	public LogLevelHandler logControl;
	JList<LogObject> logList;
	DefaultListModel<LogObject> logListModel;

	// temp vars

	// robot and field related data
	Field field = new Field(this);
	Vector<RobotPanel> robotPanels = new Vector<RobotPanel>();

	// network stuff and gui vars
	UdpListener udpListener;
	public LogLevelHandler udpLogControl;
	JList<LogObject> udpLogList;
	DefaultListModel<LogObject> udpLogListModel;
	 // load opencv
	static {
		System.loadLibrary("opencv_java249");
	}

	/**
	 * Launch the application.
	 */
	public static void main(String[] args) {
		// set local OS look and feel
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch (ClassNotFoundException | InstantiationException
				| IllegalAccessException | UnsupportedLookAndFeelException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		EventQueue.invokeLater(new Runnable() {
			public void run() {
				try {
					MainFrame frame = new MainFrame();
					frame.setVisible(true);
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the frame.
	 */
	public MainFrame() {

		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		setBounds(100, 100, 850, 466);

		JMenuBar menuBar = new JMenuBar();
		setJMenuBar(menuBar);

		JMenu mnNewMenu = new JMenu("Field View");
		menuBar.add(mnNewMenu);

		JCheckBoxMenuItem mntmEnableGrid = new JCheckBoxMenuItem("Enable Grid");
		mntmEnableGrid.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				AbstractButton aButton = (AbstractButton) e.getSource();
				boolean selected = aButton.getModel().isSelected();
				fieldPanel.gridVisible = selected;
				updateFieldIfVisible();
			}
		});
		mnNewMenu.add(mntmEnableGrid);
		contentPane = new JPanel();
		contentPane.setBorder(new EmptyBorder(5, 5, 5, 5));
		contentPane.setLayout(new BorderLayout(0, 0));
		setContentPane(contentPane);

		tabbedPane = new JTabbedPane(JTabbedPane.TOP);
		tabbedPane.setFont(new Font("Tahoma", Font.BOLD, 14));
		contentPane.add(tabbedPane, BorderLayout.CENTER);

		// --------------------------------------------------------------------
		// log tab
		JPanel logTab = new JPanel();
		tabbedPane.addTab("Log Tab", null, logTab, null);
		logTab.setLayout(new BorderLayout(0, 0));

		JScrollPane scrollPane = new JScrollPane();
		logTab.add(scrollPane);

		logList = new JList<LogObject>();
		logListModel = new DefaultListModel<LogObject>();
		logList.setModel(logListModel);
		scrollPane.setViewportView(logList);

		// --------------------------------------------------------------------
		// udp comm tab
		JPanel udpCommTab = new JPanel();
		tabbedPane.addTab("UDP Comm", null, udpCommTab, null);
		udpCommTab.setLayout(new BorderLayout(0, 0));

		JScrollPane udpScrollPane = new JScrollPane();
		udpCommTab.add(udpScrollPane);

		udpLogList = new JList<LogObject>();
		udpLogListModel = new DefaultListModel<LogObject>();
		udpLogList.setModel(udpLogListModel);
		udpScrollPane.setViewportView(udpLogList);

		// --------------------------------------------------------------------
		// initial setup
		fieldPanel = new FieldPanel(this);
		tabbedPane.insertTab("Field", null, fieldPanel, null, 0);
		tabbedPane.setSelectedIndex(0);
		for (int j = 0; j < field.robots.size(); j++) {
			fieldPanel.paintObjects.add(field.robots.get(j));
		}
		fieldPanel.paintObjects.add(field.ball);
		fieldPanel.setFocusable(true);
		fieldPanel.requestFocusInWindow();
		// --------------------------------------------------------------------
		// Robot Panel - create them
		for (int j = 0; j < field.robots.size(); j++) {
			robotPanels.add(new RobotPanel(this, field.robots.get(j)));
			tabbedPane.addTab(field.robots.get(j).name, null,
					robotPanels.lastElement(), null);
		}

		// --------------------------------------------------------------------
		// final setup
		init();

	}

	void init() {
		logControl = new LogLevelHandler(logList, logListModel, this);
		logControl.verbose = false;

		guiHelperThread = new GuiHelperThread(this, logControl);
		guiHelperThread.execute();

		field.connectRobots();

		// set the command controls on the robot panels
		for (int j = 0; j < robotPanels.size(); j++) {
			guiHelperThread.logControls.add(robotPanels.get(j).commandControl);
		}

		udpLogControl = new LogLevelHandler(udpLogList, udpLogListModel, this);
		udpLogControl.verbose = false;
		guiHelperThread.logControls.add(udpLogControl);

		udpListener = new UdpListener(this);
		udpListener.setSetting(20000);
		udpListener.start();
	}

	// robot panel related
	// functions------------------------------------------------
	RobotPanel getRobotPanelFromRobotId(int robotId) {
		if (robotId < robotPanels.size()) {
			return robotPanels.get(robotId);
		} else {
			return null;
		}
	}

	// field related functions-----------------------------------------------
	public void updateFieldIfVisible() {
		if (tabbedPane.getSelectedIndex() == 0) // if field is selected
		{
			fieldPanel.repaint();
		}
	}

	@Override
	public void exitApplication() {
		// TODO Auto-generated method stub
		System.exit(-1);
	}

}
