package constants;

public class RunConstants {
	public enum RunMode {
		TEST_TALONS (true),
		FULL (false),
		WINCH (false),
		TEST_PNEUMATICS (false),

		DRIVE_CONSTANT_PERCENT (false),
		ROTATE_TURN_MOTORS (true),
		ROTATE_TIMER (false),

		LED_TEST (true),
		JETSON_TEST (true),
		AUTO_TEST (false),
		GYRO_TEST (true),
		
		TESTING_SLOW_SPEED_PID (false),
		
		TUNE_TURN_PID (false),
		SWERVE_TESTER (false),
		KARL_TEST (true);
		
		public boolean isTest;
		
		RunMode (boolean pIsTest)
		{
			isTest = pIsTest;
		}
		
	}

	public static final RunMode RUN_MODE = RunMode.FULL;
	public static boolean 
	RUNNING_PNEUMATIC_GRABBER = true, 
	RUNNING_PNEUMATIC_SHIFTER = true,
	RUNNING_PNEUMATICS = true,
	SPEED_PID = true, 
	DRIFT_COMPENSATION = true,
	IS_PROTOTYPE_BOT = false,
	INITIALIZE_WHEEL_PIDS = false,
	
	USING_LEFT_SONAR = true,
	USING_RIGHT_SONAR = false,
	
	USING_TK1_VISION = false,
	
	USING_CUSTOM_JOYSTICK = false,
	USING_MACROS = true,
	
	EMERGENCY_TANK = false,
	LOG_USING_NY_VERSION = true,
	
	USING_AVERAGE_VOLTAGE_FOR_PRESSURE = false,
	DRIVE_ENCODER_FOR_3 = false,
	
	USING_TX1_VISION = true,
	
	RUNNING_VELOCITY_ESTIMATOR = false,
	RUNNING_DISPLACEMENT_ESTIMATOR = true,
	
	USING_DRIVE_ENCODERS_FOR_PREDICTION = false,
	
	RUN_COMPRESSOR_ALWAYS = false,
	
	DRIVE_DOWN_FIELD_POV = false,
	
	DONT_RUN_THREADS = false;
}