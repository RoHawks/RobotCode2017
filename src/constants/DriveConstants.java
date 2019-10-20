package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DriveConstants {
	public static final double RAW_TO_RPS = (1.0 / 60.0) * (34.0/70.0);
	public static final Value 
	SLOW_SHIFT_DIR = Value.kForward, 
	FAST_SHIFT_DIR = Value.kReverse;

	//speed mins, when lower than these don't do anything
	public static final double 
	MIN_ANGULAR_VEL = 0.01,
	MIN_ANGULAR_VEL_STICK = 0.02,
	MIN_LINEAR_VEL = 0.02,
	MIN_DIRECTION_MAG = 0.5, //refers to left joystick magnitude for choosing swerve direction
	MAX_INDIVIDUAL_VELOCITY = 1.0;

	public static final int MIN_TIME_IN_RANGE_NUDGE_MILLIS = 300;
	public static final double NUDGE_ANGLE_TOLERANCE = 6;

	public static final double
	MAX_RPS_FAST_MODE = 12,
	MAX_RPS_SLOW_MODE = 6;

	public static final boolean USING_DRIFT_COMP_PID = true;

	public static final double
	MAX_VOLTAGE = 12.0,
	VOLTAGE_RAMP_MILLIS = 25;

	public static final double 
	EMERGENCY_VOLTAGE = -1,
	MAX_VOLTAGE_EMERGENCY = 12.0;

	public static class SwerveSpeeds
	{
		public static double MULT_SLOW_STICK_ANGLE = 0.6;
		public static double MULT_SLOW_STICK_DRIVE = 0.6;

		public static class Real
		{
			public static double 
			SPEED_MULT = 1.0,
			ANGULAR_SPEED_MULT = 0.4,

			NUDGE_MOVE_SPEED = 0.4,
			NUDGE_TURN_SPEED = 0.4;
		}

		public static class Prototype
		{
			public static final double 
			SPEED_MULT = 1.0,
			ANGULAR_SPEED_MULT = 1.0,

			NUDGE_MOVE_SPEED = 0.5,
			NUDGE_TURN_SPEED = 0.5;
		}
	}

	public static class Modules
	{
		public static class Real
		{
			public static final boolean[] 
					TURN_ENCODER_REVERSED = new boolean[] {false, false, false, false},
					TURN_MOTOR_REVERSED = new boolean[] {false, false, false, false},
					DRIVE_ENCODER_REVERSED = new boolean[] {false, false, false, false},
					DRIVE_MOTOR_REVERSED = new boolean[] {false, false, true, false};

			public static final double[] 
					X_OFF = new double[] {19.0/2.0, 19.0/2.0, -19.0/2.0, -19.0/2.0},
					Y_OFF = new double[] {-22.0/2.0, 22.0/2.0, 22.0/2.0, -22.0/2.0};

			//			public static final double[] 
			//					ENCODER_OFFSETS = new double[] {106.3, 339.0, 120.0, 169.5};

			public static final double[] 
					ENCODER_OFFSETS = new double[] {324, 127.5 /*215*/, 227.8, 167.7 /*269.8*/};
		}

		public static class Prototype
		{
			public static final boolean[]
					TURN_ENCODER_REVERSED = new boolean[] {true, true, true, true},
					TURN_MOTOR_REVERSED = new boolean[] {false, false, false, false},
					DRIVE_MOTOR_REVERSED = new boolean[] {true, true, true, true};		

			public static final double[] 
					//X_OFF = new double[] {-9.5, -9.5, 8.5, 8.5},
					//Y_OFF = new double[] {10, -10, -9.75, 9.75};
					X_OFF = new double[] {9.5, 9.5, -8.5, -8.5},
					Y_OFF = new double[] {-10, 10, 9.75, -9.75};

			public static final double[] 
					//ENCODER_OFFSETS = new double[] {77, 181.5, 194.3, 13.2};
					ENCODER_OFFSETS = new double[] {196, 188, 255, 187};

		}
	}

	public static class PID_Constants
	{
		public static final double
		MAX_ANGULAR_VELOCITY_COMPENSATE = 2,
		TIME_AFTER_TURNING_ACTIVATE_MILLIS = 500;

		public static class Real
		{
			//			public static final double[] 
			//					ROTATION_P = new double[] {0.015, 0.015, 0.015, 0.02},
			//					ROTATION_I = new double[] {0.002, 0.002, 0.002, 0.002},
			//					ROTATION_D = new double[] {0.01, 0.01, 0.01, 0.01},
			//					ROTATION_IZONE = new double[] {15, 15, 15, 15};
			public static final double[] 
					ROTATION_P = new double[] {0.015, 0.015, 0.015, 0.015},
					ROTATION_I = new double[] {0.0, 0.0, 0.0, 0.0},
					ROTATION_D = new double[] {0.0, 0.0, 0.0, 0.0},
					ROTATION_IZONE = new double[] {15, 15, 15, 15};

			public static  double[] 
					/*FAST_SPEED_P = new double[] {0.124, 0.124, 0.124, 0.124},*/
					FAST_SPEED_P = new double[] {0.000, 0.000, 0.000, 0.000},
					FAST_SPEED_I = new double[] {0.000, 0.000, 0.000, 0.000},
					FAST_SPEED_D = new double[] {0.000, 0.000, 0.000, 0.000},
					//FAST_SPEED_F = new double[] {0.103, 0.103, 0.105, 0.109}; //after ny
					FAST_SPEED_F = new double[] {0.099, 0.101, 0.100, 0.100}; //free speed champs
					//FAST_SPEED_F = new double[] {0.096, 0.096, 0.098, 0.100}; //CM scaled champs
			
			public static  double[] 
					/*SLOW_SPEED_P = new double[] {0.124, 0.124, 0.124, 0.124},*/
					SLOW_SPEED_P = new double[] {0.000, 0.000, 0.000, 0.000},
					SLOW_SPEED_I = new double[] {0.000, 0.000, 0.000, 0.000}, //be careful
					SLOW_SPEED_D = new double[] {0.000, 0.000, 0.000, 0.000},
					//SLOW_SPEED_F = new double[] {0.179, 0.19, 0.19, 0.185}; //after ny
					SLOW_SPEED_F = new double[] {0.200, 0.200, 0.200, 0.200}; //free speed champs
					//SLOW_SPEED_F = new double[] {0.195, 0.195, 0.197, 0.200}; //CM scaled champs

			public static final double 
			ANGLE_SLOW_P = 0.02, //0.0178
			ANGLE_SLOW_I = 0.0, //0.005,
			ANGLE_SLOW_D = 0.0, //0.04,
			ANGLE_SLOW_IZONE = 8,
			ANGLE_SLOW_MAX = 0.5,
			ANGLE_SLOW_MIN = 0.0;

			public static final double 
			ANGLE_FAST_P = 0.007,
			ANGLE_FAST_I = 0.00, //0.005,
			ANGLE_FAST_D = 0.0, //0.04,
			ANGLE_FAST_IZONE = 5,
			ANGLE_FAST_MAX = 0.5,
			ANGLE_FAST_MIN = 0.0;

			public static final double
			ANGLE_DEADBAND = 0;

			public static final double 
			DRIFT_COMPENSATION_ZONE_1 = 0.4,
			DRIFT_COMPENSATION_ZONE_2 = 0.7,
			DRIFT_COMPENSATION_P1 = 0.2,
			DRIFT_COMPENSATION_P2 = 0.5,
			DRIFT_COMPENSATION_P3 = 1.0;

			public static final double
			DRIFT_COMP_FAST_TARGET_P = 0.02,
			DRIFT_COMP_FAST_TARGET_I = 0.0002,
			DRIFT_COMP_FAST_TARGET_MAX = 0.3;

			public static final double
			DRIFT_COMP_SLOW_TARGET_P = 0.002,
			DRIFT_COMP_SLOW_TARGET_I = 0.0,
			DRIFT_COMP_SLOW_TARGET_MAX = 0.0;

		}

		public static class Prototype
		{
			public static final double[] 
					ROTATION_P = new double[] {0.02, 0.02, 0.02, 0.02},
					ROTATION_I = new double[] {0.000, 0.000, 0.000, 0.000},
					ROTATION_D = new double[] {0, 0, 0, 0},
					ROTATION_IZONE = new double[] {15, 15, 15, 15};

			//for turning robot to an angle
			public static final double 
			ANGLE_P = 0.025,
			ANGLE_I = 0.005,
			ANGLE_D = 0.04,
			ANGLE_IZONE = 5,
			ANGLE_MAX = 0.4,
			ANGLE_MIN = 0.1,
			ANGLE_DEADBAND = 2;

			public static final double 
			DRIFT_COMPENSATION_ZONE_1 = 0.4,
			DRIFT_COMPENSATION_ZONE_2 = 0.7,
			DRIFT_COMPENSATION_P1 = 0.2,
			DRIFT_COMPENSATION_P2 = 0.5,
			DRIFT_COMPENSATION_P3 = 1.0;

			public static final double
			DRIFT_COMP_TARGET_P = 0.04,
			DRIFT_COMP_TARGET_I = 0.001,
			DRIFT_COMP_MAX = 0.3;
		}
	}



}
