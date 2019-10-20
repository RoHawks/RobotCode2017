package resource;

public class ResourceFunctions {
	public static double putAngleInRange(double angle) {
		return (angle + 360000000) % 360;
	}
	
	public static double continuousAngleDif (double angle1, double angle2) {
		double dif = putAngleInRange(angle1) - putAngleInRange(angle2);
		dif = putAngleInRange(dif);
		//if (dif > 180) dif -= 180;
		if (dif > 180) dif = dif - 360;
		return dif;
	}

	public static boolean equals(double a, double b) {
		return (Math.abs(a - b) < 0.001);
	}

	public static double PutNumInAbsoluteRange(double val, double min,
			double max) {
		
		if (val > max)
		{
			return max;
		}
		else if (val < min)
		{
			return min;
		}
		else
		{
			return val;
		}
	}
}
