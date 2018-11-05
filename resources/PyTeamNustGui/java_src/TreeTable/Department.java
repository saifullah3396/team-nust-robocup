package TreeTable;

import java.util.List;

public class Department {

    private int id;

	private String name;
    private List<Employee> employeeList;

    public Department(int id, String name, List<Employee> empList) {
        this.id = id;        
        this.employeeList = empList;
    }

    public List<Employee> getEmployeeList() {
        return employeeList;
    }

    public void setEmployeeList(List<Employee> employeeList) {
        this.employeeList = employeeList;

    }
    //other setters and getters
    
    public int getId() {
		return id;
	}

	public void setId(int id) {
		this.id = id;
	}

	public String getName() {
		return name;
	}

	public void setName(String name) {
		this.name = name;
	}

}
