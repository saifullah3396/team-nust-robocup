package DataContainers;

import java.util.Vector;

public class Memory {

	String name ;
	public Vector<Variable> variables = new Vector<Variable>();
	
	Memory(String name)
	{
		this.name= name;
	}
	
	public String toString()
	{
		String str = name + ":";
		str += "{";
		int limit = variables.size() - 1;
		for(int j=0; j<variables.size(); j++)
		{
			str+= variables.get(j).toString();
			
			if(j != limit)
			{
				str+= ",";
			}
		}
		
		return str += "}";
	}
}
