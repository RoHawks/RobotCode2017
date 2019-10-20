package constants;

public class ZedAlignmentConstants {
	public static final double
	LR_P = 0.05,
	LR_I = 0.00,
	LR_D = 0.0,
	LR_MAX = 0.2;

	public static final double
	ANGLE_P = 0.02,
	ANGLE_I = 0.0,
	ANGLE_D = 0.0,
	ANGLE_MAX = 0.1;

	public static final double 
	LR_SETPOINT = 0.0,
	ANGLE_SETPOINT = 0.0;

	public static final double
	FORWARD_SPEED = 0.2;

	public static final long 
	TIME_FORWARD_MILLIS = 1000;

	public static final double
	HUMAN_MULTIPLIER = 0.3;
	
	public static final String[] RobotPositionData = new String[]
			{
					"RobotLR",
					"RobotFB",
					"RobotAngle",
					"RobotTime",
					"ZedTime",
					"TimeDiff"
			};

	public static final double LR_INITIAL_TOLERANCE = 4.0;
	public static final double ANGLE_INITIAL_TOLERANCE = 3.0;
	
	public static final int MILLIS_TO_FORGET_POSE = 1000;
	
	public static final double RANGE_START_COUNTING = 30;
	
	public static final double MIN_DIF_ERASE_ESTIMATE = 10;
}
