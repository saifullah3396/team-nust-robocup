package TreeTable;

import java.util.*;
import javax.swing.*;
import org.jdesktop.swingx.JXTreeTable;

public class TreeTableTest extends JFrame {

    private JXTreeTable treeTable;

    public TreeTableTest() {
        //sample doj
        final Date doj = Calendar.getInstance().getTime();        
        List<Department> departmentList = new ArrayList<Department>();

        List<Employee> empList = new ArrayList<Employee>();
        empList.add(new Employee(1, "Ram", doj, "emp1.jpg"));
        empList.add(new Employee(2, "Krishna", doj, "emp2.jpg"));

        //create and add the first department with its list of Employee objects
        departmentList.add(new Department(1, "Sales", empList));

        List<Employee> empList2 = new ArrayList<Employee>();
        empList2.add(new Employee(3, "Govind", doj, "emp1.jpg"));
        empList2.add(new Employee(4, "Kiran", doj, "emp2.jpg"));
        empList2.add(new Employee(5, "Prabhu", doj, "emp1.jpg"));

        //create and add the first department with its list of Employee objects
        departmentList.add(new Department(2, "Marketing", empList2));
        
        //we use a no root model
        NoRootTreeTableModel noRootTreeTableModel = new NoRootTreeTableModel(departmentList);
        treeTable = new JXTreeTable(noRootTreeTableModel);
        treeTable.setAutoResizeMode(JTable.AUTO_RESIZE_OFF);        
        treeTable.setRootVisible(false);  // hide the root

        add(new JScrollPane(treeTable));

        setTitle("JXTreeTable Example");
        setDefaultCloseOperation(EXIT_ON_CLOSE);
        pack();
        setVisible(true);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {        
                new TreeTableTest();
            }
        });
    }
}