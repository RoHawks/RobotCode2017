package constants;

public class CamAlignerConstants {
	public static final String 
	TABLE_NAME = "SmartDashboard",
	IS_VALID_KEY = "IsValid2",
	
	UL_X_KEY = "UL_X",
	UL_Y_KEY = "UL_Y",
	
	UR_X_KEY = "UR_X",
	UR_Y_KEY = "UR_Y",
	
	LL_X_KEY = "LL_X",
	LL_Y_KEY = "LL_Y",

	LR_X_KEY = "LR_X",
	LR_Y_KEY = "LR_Y";

	public static final double 
	LR_SETPOINT = 0.43,
	DISTANCE_SETPOINT_INITIAL = 4; //min distance before putting gear on peg

	public static final long
	MIN_TIME_ALIGNED = 500;

	public final static boolean 
	ACTUALLY_RUN_VISION = true,
	SHOULD_NUDGE_FORWARD = true;

	public static class PID_Constants
	{
		public static class Real
		{
			public static final double
			LR_P = 2.0,
			LR_I = 0.0,
			LR_D = 0.0,
			LR_MAX = 0.2,
			LR_MIN = 0.1,
			LR_DEADBAND = 0.0;

			public static final double
			ANGLE_P = 0.04,
			ANGLE_I = 0.0,
			ANGLE_D = 0.0,
			ANGLE_MAX = 0.12,
			ANGLE_MIN = 0.0,
			ANGLE_DEADBAND = 0.0;

			public static final double
			FORWARD_VEL_INITIAL = 0.4,
			FORWARD_VEL_FINAL = 0.4,
			TIME_FORWARD = 1000,
			TIME_BACKWARD = 2000;

			public static final double
			LR_TOLERANCE = 0.05,
			ANGLE_TOLERANCE = 10;
		}
		
		public static class Prototype
		{
			public static final double
			LR_P = 6.0,
			LR_I = 0.1,
			LR_D = 0.0,
			LR_MAX = 0.25,
			LR_MIN = 0.0,
			LR_IZONE = 0.05,
			LR_DEADBAND = 0.0;

			public static final double
			ANGLE_P = 0.05,
			ANGLE_I = 0.005,
			ANGLE_D = 0.0,
			ANGLE_MAX = 0.5,
			ANGLE_MIN = 0.0,
			ANGLE_DEADBAND = 0.0;

			public static final double
			FORWARD_VEL_INITIAL = 0.3,
			FORWARD_VEL_FINAL = 0.4,
			TIME_FORWARD = 1800,
			TIME_BACKWARD = 2000;

			public static final double
			LR_TOLERANCE = 0.01,
			ANGLE_TOLERANCE = 2;
		}
	}
}
