package constants;

public class Configurables {
	public static final double ZERO_RANGE = 0.02; //for certain joystick utilities
	public static final double ERROR_MIN = 10000; //for certain PID outputs, higher than this value tells the code not to use it

	public static final double GYRO_SAMPLE_TIME = 5.0;
	
	public static final int TX1_BYTES = 64;
	
	public static String[] LOGGER_COMPONENTS = new String[]
			{
					"TimeElapsed",
					"NE Drive",
					"NE Turn",
					"NW Drive",
					"NW Turn",
					"SW Drive",
					"SW Turn",
					"SE Drive",
					"SE Turn",
					"Gear Aligner",
					"Winch",
					"Compressor",
					"Total Current",
					"Total Voltage",
					"Gear Breakbeam",
					"Gear Limitswitch",
					"Gyro Angle",
					"Compressor On",
					"Pressure",
					
					"POV Mode",
					
					"NE Desired Angle",
					"NE Real Angle",
					"NE Percent Vbus",
					"NW Desired Angle",
					"NW Real Angle",
					"NW Percent Vbus",
					"SW Desired Angle",
					"SW Real Angle",
					"SW Percent Vbus",
					"SE Desired Angle",
					"SE Real Angle",
					"SE Percent Vbus",
					
					"Desired Robot Vel Angle",
					"Desired Robot Vel Magnitude",
					"Desired Angular Vel",
					
					"Fast Gear",
					
					"Grabber State",
					
					"Xbox Y",
					"Xbox B",
					"Xbox A",
					"Xbox X",
					"Left Bumper",
					"Right Bumper",
					"Left Trigger",
					"Right Trigger",
					"DPad",
					"Start Button",
					"Back Button",
					"Left Joystick X",
					"Left Joystick Y",
					"Right Joystick X",
					"Right Joystick Y",
					
					"Left Station Button",
					"Middle Station Button",
					"Right Station Button",
					"Cycle Placement Button",
					"Flap Flipper Button",
					"Emergency Up Button",
					"Automatic Climb Button",
					"Manual Climb Button",
					"Reverse Dir Button",
					
					"Loop Time Millis"
			};
	
	public static String[] WheelTestHeader = new String[] {
		"Timestamp",
		"BatteryVoltage",
		"DesiredVoltage",
		"NE_Voltage",
		"NW_Voltage",
		"SW_Voltage",
		"SE_Voltage",
		"NE_Current",
		"NW_Current",
		"SW_Current",
		"SE_Current",
		"NE_Speed",
		"NW_Speed",
		"SW_Speed",
		"SE_Speed",
	};
}
