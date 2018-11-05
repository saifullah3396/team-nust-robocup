package DataContainers;

import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;
import java.util.Vector;

public class Utilities {

	
	static Point2D.Double PolarToCartesian(double mag, double angle)
	{
		return new Point2D.Double(mag * Math.cos(angle), mag * Math.sin(angle));
	}
	
	static Point2D.Double CartesianToPolar(Point2D.Double pt)
	{
		return new Point2D.Double( Math.sqrt(pt.distanceSq(0,0)), Math.atan2(pt.y, pt.x) );
	}
	
	public static Vector<java.lang.Double> ParseStringToVector(String str)
	{
		if(str.length() == 0)
		{
			return null;
		}
		
		Vector<java.lang.Double> vect = new Vector<java.lang.Double>();
		String parts[] = str.substring(1, str.length() - 1).split(",");
		for(int j=0; j<parts.length; j++)
		{
			if(parts[j].equals("nan"))
			{
				vect.add(java.lang.Double.MAX_VALUE);
			}else if(parts[j].equals("-nan"))
			{
				vect.add(java.lang.Double.MIN_VALUE);
			}else
			{
				vect.add(java.lang.Double.parseDouble(parts[j]));
			}
			
		}
		
		return vect;
	}
	
	public static byte[] hexStringToByteArray(String s) {
	    int len = s.length();
	    byte[] data = new byte[len / 2];
	    if(len % 2 == 0)
	    { 
		    for (int i = 0; i < len; i += 2) {
		        data[i / 2] = (byte) ((Character.digit(s.charAt(i), 16) << 4)
		                             + Character.digit(s.charAt(i+1), 16));
		    }
	    }
	    return data;
	}
}
