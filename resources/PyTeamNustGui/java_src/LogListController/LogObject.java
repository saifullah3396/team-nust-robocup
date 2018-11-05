package LogListController;

public class LogObject {
	int priorityLevel;
	String data;
	int messageId;
	
	public LogObject(int priorityLevel2, String message) {
		// TODO Auto-generated constructor stub
		priorityLevel = priorityLevel2;
		data = message;
	}
	

	public LogObject(int priorityLevel2, String message, int msgId) {
		// TODO Auto-generated constructor stub
		priorityLevel = priorityLevel2;
		data = message;
		messageId = msgId;
	}
	
	public int getPriorityLevel()
	{
		return priorityLevel;
	}

	public String getData()
	{
		return data;
	}
}
