package constants;

public class UltrasonicAlignerConstants {
	public static final double
	TOLERANCE_MIDDLE = 0.25,
	TOLERANCE_SIDE = 0.375, //0.375,
	MIN_TIME_IN_RANGE = 200,
	MAX_RANGE_INCHES = 17,
	PID_SETPOINT = 8.02;

	public static class Prototype
	{
		public static final double
		PID_P = 0.09,
		PID_I = 0.008,
		PID_D = 0.0,
		PID_MAX = 0.3,
		PID_IZONE = 3.0;
	}

	public static class Real
	{
		public static final double
		PID_P = 0.10, ////0.07, //0.05, //0.25,
		PID_I = 0.003, //0.002, //0.002,
		PID_D = 0.0,
		PID_MAX = 0.12,
		PID_IZONE = 2.0;
	}
}
