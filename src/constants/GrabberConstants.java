package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class GrabberConstants {
	//pneumatic states
	public static final Value   
	FRAME_UP = Value.kReverse,
	FRAME_DOWN = Value.kForward,
	CLAMPER_OPEN = Value.kReverse,
	CLAMPER_CLOSED = Value.kForward,
	SNATCHER_OPEN = Value.kForward,
	SNATCHER_CLOSED = Value.kReverse;

	//break beam normally open or closed
	public static final boolean GEAR_PRESENT_BREAK_BEAM = false, GEAR_PRESENT_LIMIT_SWITCH = true;

	//time to activate pistons
	public static final double 
	FRAME_TIME_DOWN = 0.5,
	FRAME_TIME_UP = 0.3,
	CLAMPER_TIME_OPEN = 0.3,
	CLAMPER_TIME_CLOSE = 0.5,
	SNATCHER_TIME_RELEASE_PRESSURE_OPEN = 0.04,
	SNATCHER_TIME_RELEASE_PRESSURE_CLOSE = 0.12,
	SNATCHER_TIME_OPEN = 0.3,
	SNATCHER_TIME_CLOSE = 0.3,
	WAIT_AFTER_ALIGNED = 0.2,
	WAIT_AFTER_BUMPSWITCH = 0.2;

	public static final long TIME_TO_BAIL = 3000;
	public static final long TIME_FLAP_PRIYA = 200;
}
