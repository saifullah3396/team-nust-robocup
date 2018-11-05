package DataContainers;

import java.util.Vector;

public class Variable {
	public int index;
	public String name;
	public String value;
	
	public Variable()
	{
		
	}
	
	public Variable(int index, String name)
	{
		this.index = index;
		this.name = name;
	}
	
	public Variable(int index, String name, String value)
	{
		this.index = index;
		this.name = name;
		this.value = value;
	}
	
	public String toString()
	{
		String str = Integer.toString(index) + ":" + name + ":";
		str+= value;		
		return str;
	}
}
