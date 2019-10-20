package constants;

public class Ports {
	public static class Real
	{
		public static final int[] 
				TURN = new int[] {0, 1, 2, 3},
				//DRIVE = new int[] {4, 5, 6, 7};
				DRIVE = new int[] {4, 5, 8, 7};

		public static final int 
		SNATCHER_PISTON = 6,
		CLAMPER_PISTON = 7,

		FRAME_PISTON_IN = 4,
		FRAME_PISTON_OUT = 5,

		SHIFTER_SOLENOID_IN = 2, 
		SHIFTER_SOLENOID_OUT = 3;

		public static final int GEAR_ALIGNER = 10, WINCH_MOTOR = 11;

		public static final int BREAK_BEAM = 9, LIMIT_SWITCH = 8;
		public static final int LED_DATA = 1, LED_CLOCK = 0;
		public static final int WINCH_ENCODER_A = 18, WINCH_ENCODER_B = 19;
		public static final int WINCH_BREAKBEAM = 7;
		public static final int 
		ECHO_LEFT = 6, 
		PING_LEFT = 5, 
		ECHO_RIGHT = 20, 
		PING_RIGHT = 21;

		public static final int PRESSURE_SENSOR = 3;
	}

	public static class Prototype
	{

		//		public static final int[] ENCODERS = new int[] {3, 2, 1, 0};
		//		public static final int[] TURN = new int[] {0, 2, 5, 6};
		//		public static final int[] DRIVE = new int[] {1, 3, 4, 7};
		public static final int[] ENCODERS = new int[] {1, 0, 3, 2};
		public static final int[] TURN = new int[] {5, 6, 0, 2};
		public static final int[] DRIVE = new int[] {4, 7, 1, 3};

		public static final int 
		FRAME_PISTON_IN = 0,
		FRAME_PISTON_OUT = 1,
		CLAMPER_PISTON_IN = 2,
		CLAMPER_PISTON_OUT = 3,
		SNATCHER_PISTON_IN = 4,
		SNATCHER_PISTON_OUT = 5;

		public static final int BREAK_BEAM = 8, LIMIT_SWITCH = 9;
		public static final int GEAR_ALIGNER = 9, WINCH_MOTOR = 8;
		public static final int WINCH_ENCODER_A = 6, WINCH_ENCODER_B = 7;
		public static final int WINCH_BREAKBEAM = 5;
		public static final int ECHO_LEFT = 23, PING_LEFT = 22, ECHO_RIGHT = 20, PING_RIGHT = 21;
	}
}
