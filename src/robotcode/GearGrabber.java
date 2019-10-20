
package robotcode;


import constants.ControlPorts;
import constants.GrabberConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearGrabber implements Runnable {

	private boolean mEnabled;
	private SolenoidInterface mClamper, mFrame, mSnatcher;
	private DigitalInput mBreakBeam, mLimitSwitch;
	private GearState mGearState;
	
	private Joystick mStick;
	
	private long mTimeStartedFlapping;
	private int mFrameNum;
	private boolean mAutoWantsRelease, mAutoWantsUp, mAutoWantsDown;
	
	private SpeedController mSquishyWheel;
	
	private static final int SEARCHING_NUM = 0, GRABBING_NUM = 1, GRABBED_NUM = 2;
	
	public enum GearState {
		BEGIN_STATE,
		NONE,
		FLAPPING_OUT,
		FLAPPING_IN,
		GRABBING,
		GRABBED,
		FRAME_UP,
		READY_TO_RELEASE,
		RELEASED,
		READY_TO_DOWN,
		EMERGENCY_UP
	}
	
	public GearGrabber (
			DigitalInput pBreakBeam, DigitalInput pLimitSwitch, 
			SolenoidInterface pFrame, SolenoidInterface pClamper, 
			SolenoidInterface pSnatcher,
			Joystick pStick) 
	{
		mStick = pStick;
		mEnabled = true;
		
		mBreakBeam = pBreakBeam;
		mLimitSwitch = pLimitSwitch;
		
		mFrame = pFrame;
		mClamper = pClamper;
		mSnatcher = pSnatcher;
		
		mGearState = GearState.BEGIN_STATE;
		
		mFrame.set (GrabberConstants.FRAME_UP);
		mClamper.set (GrabberConstants.CLAMPER_OPEN);
		mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
		
		mFrameNum = 0;
		
		mAutoWantsRelease = false;
		mAutoWantsUp = false;
		mAutoWantsDown = false;
	}
	
	public void setSquishyWheel (SpeedController pSquishyWheel)
	{
		mSquishyWheel = pSquishyWheel;
		mSquishyWheel.set(0.0);
	}
	
	public synchronized GearState getGearState()
	{
		return mGearState;
	}
		
	public synchronized boolean getGearUp()
	{
		return mGearState == GearState.READY_TO_RELEASE;
	}
	
	public synchronized boolean getGearDown()
	{
		return mGearState == GearState.NONE;
	}
	
	public synchronized boolean getGearReleased()
	{
		return mGearState == GearState.READY_TO_DOWN;
	}
	
	public synchronized void setGearUp (boolean pGearUp)
	{
		mAutoWantsUp = pGearUp;
	}
	
	public synchronized void setReleaseGear(boolean pReleaseGear)
	{
		mAutoWantsRelease = pReleaseGear;
	}
	
	public synchronized void setShouldDown (boolean pDown)
	{
		mAutoWantsDown = pDown;
	}
	
	private boolean snatchGear() 
	{
		mSnatcher.set(GrabberConstants.SNATCHER_CLOSED);
		Timer.delay(GrabberConstants.SNATCHER_TIME_CLOSE);
		
		boolean consistentlyHittingBumpswitch = false;
		
		while (!consistentlyHittingBumpswitch && !Thread.interrupted()) {
			//open and close until it works
			long timeStarted = System.currentTimeMillis();
			while (mLimitSwitch.get() != GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH && !Thread.interrupted()) {
				mSnatcher.set(GrabberConstants.SNATCHER_OPEN);
				Timer.delay(GrabberConstants.SNATCHER_TIME_RELEASE_PRESSURE_OPEN);
				mSnatcher.set(GrabberConstants.SNATCHER_CLOSED);
				Timer.delay(GrabberConstants.SNATCHER_TIME_RELEASE_PRESSURE_CLOSE);
				
				if (System.currentTimeMillis() - timeStarted > GrabberConstants.TIME_TO_BAIL ||
					mStick.getRawButton(ControlPorts.EMERGENCY_OPEN_PORT)) {
					return false;
				}
			}
			
			Timer.delay (GrabberConstants.WAIT_AFTER_ALIGNED);
			if (mLimitSwitch.get() == GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH) {
				consistentlyHittingBumpswitch = true;
			}
			else {
				consistentlyHittingBumpswitch = false;
			}
		}
		
		
//		while (Math.abs(mXbox.getTriggerAxis(Hand.kRight)) < 0.3 && mEnabled && !Thread.interrupted()){
//			Timer.delay(0.005);
//		}
		

		return true;
		//wait until it hits the switch
//		while (mLimitSwitch.get() != GrabberConstants.GEAR_PRESENT_BREAK_BEAM) {
//			Timer.delay(0.005);
//		}

	}
	
	private void grabGear() {
		//set initial states
		mSnatcher.set(GrabberConstants.SNATCHER_CLOSED);
		mClamper.set(GrabberConstants.CLAMPER_OPEN);
		mFrame.set(GrabberConstants.FRAME_DOWN);
		
		mClamper.set(GrabberConstants.CLAMPER_CLOSED);
		mSnatcher.set(GrabberConstants.SNATCHER_OPEN);
		Timer.delay(Math.max(GrabberConstants.CLAMPER_TIME_CLOSE, GrabberConstants.SNATCHER_TIME_OPEN));
	}

	private void setPistonDirs()
	{
		switch (mGearState)
		{
		case BEGIN_STATE:
			mFrame.set (GrabberConstants.FRAME_DOWN);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case NONE:
			mFrame.set (GrabberConstants.FRAME_DOWN);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case EMERGENCY_UP:
			mFrame.set (GrabberConstants.FRAME_UP);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case FLAPPING_OUT:
			mFrame.set (GrabberConstants.FRAME_DOWN);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case FLAPPING_IN:
			mFrame.set (GrabberConstants.FRAME_DOWN);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_CLOSED);
			break;
		case GRABBED:
			mFrame.set (GrabberConstants.FRAME_DOWN);
			mClamper.set (GrabberConstants.CLAMPER_CLOSED);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case GRABBING: //should never happen
			mGearState = GearState.NONE;
			break;
		case FRAME_UP:
			mFrame.set (GrabberConstants.FRAME_UP);
			mClamper.set (GrabberConstants.CLAMPER_CLOSED);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case READY_TO_RELEASE:
			mFrame.set (GrabberConstants.FRAME_UP);
			mClamper.set (GrabberConstants.CLAMPER_CLOSED);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case RELEASED:
			mFrame.set (GrabberConstants.FRAME_UP);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		case READY_TO_DOWN:
			mFrame.set (GrabberConstants.FRAME_UP);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
			break;
		}
	}
	
	private void handleStates()
	{
		if (mGearState == GearState.BEGIN_STATE)
		{
			Timer.delay(1.0);
			mGearState = GearState.NONE;
		}
		
		if (mStick.getRawButton(ControlPorts.EMERGENCY_OPEN_PORT))
		{
			mGearState = GearState.EMERGENCY_UP;
		}
		
		if (mGearState == GearState.EMERGENCY_UP && !mStick.getRawButton(ControlPorts.EMERGENCY_OPEN_PORT))
		{
			mGearState = GearState.NONE;
		}
		
		//snatch the gear
		if (mBreakBeam.get() == GrabberConstants.GEAR_PRESENT_BREAK_BEAM && mGearState == GearState.NONE) {
			System.out.println("Attempting to snatch!");
			sendSDState();
			mGearState = GearState.GRABBING;
			sendSDState();
			if (!RunConstants.IS_PROTOTYPE_BOT)
			{
				mSquishyWheel.set(0.5);
			}
			if (snatchGear()) {
				sendSDState();
				grabGear();
				mGearState = GearState.GRABBED;
			}
			else
			{
				mGearState = GearState.NONE;
			}
			if (!RunConstants.IS_PROTOTYPE_BOT)
			{
				mSquishyWheel.set(0.0);
			}
			sendSDState();
		}
		
		if (mGearState == GearState.GRABBED && mAutoWantsUp)
		{
			mGearState = GearState.READY_TO_RELEASE;
		}

		if (mGearState == GearState.READY_TO_DOWN && mAutoWantsDown)
		{
			mGearState = GearState.NONE;
		}
		
		if (mAutoWantsRelease && mGearState == GearState.READY_TO_RELEASE)
		{
			mGearState = GearState.READY_TO_DOWN;
		}
		
		if (mStick.getRawButton(ControlPorts.CYCLE_PLACEMENT_PORT) && mGearState == GearState.GRABBED)
		{
			mGearState = GearState.FRAME_UP;
		}
		
		if (!mStick.getRawButton(ControlPorts.CYCLE_PLACEMENT_PORT) && mGearState == GearState.FRAME_UP)
		{
			mGearState = GearState.READY_TO_RELEASE;
		}
		
		if (mStick.getRawButton(ControlPorts.CYCLE_PLACEMENT_PORT) && mGearState == GearState.READY_TO_RELEASE) 
		{
			mGearState = GearState.RELEASED;
		}
		
		if (!mStick.getRawButton(ControlPorts.CYCLE_PLACEMENT_PORT) && mGearState == GearState.RELEASED) {
			mGearState = GearState.READY_TO_DOWN;
		}
		
		if (mStick.getRawButton(ControlPorts.CYCLE_PLACEMENT_PORT) && mGearState == GearState.READY_TO_DOWN) {
			mGearState = GearState.NONE;
		}
		
		if (mStick.getRawButton(ControlPorts.FLAP_PORT) && mGearState == GearState.NONE) {
			mGearState = GearState.FLAPPING_IN;
			mTimeStartedFlapping = System.currentTimeMillis();
		}
		
		if (!mStick.getRawButton(1) && 
				(mGearState == GearState.FLAPPING_IN || mGearState == GearState.FLAPPING_OUT)) {
			mGearState = GearState.NONE;
			mTimeStartedFlapping = -1;
		}
		
		if (mGearState == GearState.FLAPPING_IN && 
				System.currentTimeMillis() - mTimeStartedFlapping > GrabberConstants.TIME_FLAP_PRIYA) {
			mGearState = GearState.FLAPPING_OUT;
			mTimeStartedFlapping = System.currentTimeMillis();
		}
		
		if (mGearState == GearState.FLAPPING_OUT && 
				System.currentTimeMillis() - mTimeStartedFlapping > GrabberConstants.TIME_FLAP_PRIYA) {
			mGearState = GearState.FLAPPING_IN;
			mTimeStartedFlapping = System.currentTimeMillis();
		}
	}
	
	public int getGearStateNum()
	{
		int gearGrabbedForDashboard = SEARCHING_NUM;
		switch (mGearState)
		{
		case BEGIN_STATE:
			gearGrabbedForDashboard = SEARCHING_NUM;
			break;
		case NONE:
			gearGrabbedForDashboard = SEARCHING_NUM;
			break;
		case EMERGENCY_UP:
			gearGrabbedForDashboard = SEARCHING_NUM;
			break;
		case FLAPPING_IN:
			gearGrabbedForDashboard = SEARCHING_NUM;
			break;
		case FLAPPING_OUT:
			gearGrabbedForDashboard = SEARCHING_NUM;
			break;
		case GRABBED:
			gearGrabbedForDashboard = GRABBED_NUM;
			break;
		case FRAME_UP:
			gearGrabbedForDashboard = GRABBED_NUM;
			break;
		case GRABBING:
			gearGrabbedForDashboard = GRABBING_NUM;
			break;
		case READY_TO_DOWN:
			gearGrabbedForDashboard = SEARCHING_NUM;
			break;
		case READY_TO_RELEASE:
			gearGrabbedForDashboard = GRABBED_NUM;
			break;
		case RELEASED:
			gearGrabbedForDashboard = SEARCHING_NUM;
			break;
		}
		
		return gearGrabbedForDashboard;
	}
	
	private void sendSDState()
	{
		SmartDashboard.putString ("Gear State", mGearState.toString());
		SmartDashboard.putNumber ("GearGrabbed", getGearStateNum());
	}
	
	private void process() {
		mFrameNum++;
		SmartDashboard.putNumber ("Frame Num", mFrameNum);
		
		
		if (mGearState != GearState.GRABBED)
		{
			mAutoWantsUp = false;
		}
		if (mGearState != GearState.READY_TO_RELEASE)
		{
			mAutoWantsRelease = false;
		}
		if (mGearState != GearState.READY_TO_DOWN)
		{
			mAutoWantsDown = false;
		}
		
		
		sendSDState();
		setPistonDirs();	
		handleStates();
	}
	
	public synchronized void disable() {
		mEnabled = false;
	}
	
	public synchronized void enable() {
		mEnabled = true;
	}
	
	@Override
	public void run() {
		while (mEnabled && !Thread.interrupted()) {
			process();
			Timer.delay(0.05);
		}
		mFrame.set (GrabberConstants.FRAME_UP);
		mClamper.set (GrabberConstants.CLAMPER_OPEN);
		mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
		if (!RunConstants.IS_PROTOTYPE_BOT)
		{
			mSquishyWheel.set(0.0);
		}
	}
	
}
