package robotcode.driving;

import constants.Configurables;
import constants.ControlPorts;
import constants.DriveConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LocalPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import robotcode.SolenoidInterface;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;
import robotcode.pid.GenericPIDOutput;
import robotcode.pid.PersistentlyInRange;
import sensors.RobotAngle;

/**
 * Controls the swerve drive in all control modes
 * Uses the XbxoController and visual feedback to determine 
 * high-level state decisions such as robot's linear and angular velocity
 * Individual wheel velocities are then calculated and achieved seperately
 * @author 3419
 *
 */
public class DriveTrain {
	private SwerveDrive mSwerveDrive;
	private Wheel[] mWheels;
	private RobotAngle mRobotAngle;

	private LocalPIDController mGyroPID;
	private GenericPIDOutput mGyroPID_Output;

	private XboxController mController;
	private Joystick mSecondaryStick;

	private SolenoidInterface mShifterSolenoid;
	private long mTimeStoppedTurning;

	private DirectionMode mPOVMode;
	private boolean mSlowStick;

	private LocalPIDController mDriftCompensationPID;
	private GenericPIDOutput mDriftCompensationPID_Output;
	
	private Vector mDesiredRobotVel;
	private double mDesiredAngularVel;

	public DriveTrain (Wheel[] pWheels, 
			RobotAngle pRobotAngle,
			SolenoidInterface pShifterSolenoid, 
			XboxController pController, 
			Joystick pSecondaryStick) {
		mTimeStoppedTurning = -1;

		mSwerveDrive = new SwerveDrive(pWheels);
		mWheels = pWheels;
		mRobotAngle = pRobotAngle;

		mController = pController;
		mSecondaryStick = pSecondaryStick;

		mShifterSolenoid = pShifterSolenoid;

		//initialize gyro PID
		mGyroPID_Output = new GenericPIDOutput();
		mGyroPID = new LocalPIDController(
				0, 0, 0, 
				mRobotAngle, mGyroPID_Output);

		mGyroPID.setContinuous(true);
		mGyroPID.setInputRange(0, 360);
		mGyroPID.setNegate(false);
		mGyroPID.setSetpoint(0);
		mGyroPID.enable();
		setGyroPIDGains(false);


		mPOVMode = DirectionMode.FIELD_RELATIVE;
		mSlowStick = false;

		mDriftCompensationPID_Output = new GenericPIDOutput();
		mDriftCompensationPID = new LocalPIDController(
				0, 0, 0,
				mRobotAngle, mDriftCompensationPID_Output);
		mDriftCompensationPID.setContinuous(true);
		mDriftCompensationPID.setInputRange(0, 360);
		mDriftCompensationPID.setNegate(false);
		mDriftCompensationPID.setSetpoint(0);
		mDriftCompensationPID.disable();
		setDriftCompensationPIDGains(true);
		
		mDesiredRobotVel = new Vector();
		mDesiredAngularVel = 0;
	}
	
	private enum DirectionMode
	{
		FIELD_RELATIVE,
		POV_FORWARD,
		POV_BACKWARD
	}

	private enum AngularVelState {
		NUDGE (false),
		STICK (false),
		TURN_TO_0 (true),
		TURN_TO_90 (true),
		TURN_TO_180 (true),
		TURN_TO_270 (true),

		TURN_TO_45 (true),
		TURN_TO_135 (true),
		TURN_TO_225 (true),
		TURN_TO_315 (true),

		TURN_TO_60 (true),
		TURN_TO_300 (true),

		TURN_TO_JOYSTICK_DIR (true),
		//ALIGN_PEG (true),
		NONE (false);

		boolean isGyroPID;

		AngularVelState (boolean pIsGyroPID) 
		{
			isGyroPID = pIsGyroPID;
		}
	}

	private enum RobotVelState {
		NUDGE,
		STICK,
		PREANGLE,
		NONE
	}

	public enum SpeedMode {
		SLOW,
		FAST,
		INDIFFERENT;
	}

	public static class SwerveMovement
	{
		public double mAngularVel;
		public Vector mRobotVel;
		public SwerveState mSwerveState;
		public boolean mNudge;
		public SpeedMode mSpeedMode;

		public enum SwerveState
		{
			NORMAL,
			HOLD_FOR_DIRECTION,
			STAY_STILL,
			DO_NOTHING;
		}

		public SwerveMovement (
				double pAngularVel,
				Vector pRobotVel, 
				SwerveState pSwerveState,
				boolean pNudge,
				SpeedMode pSpeedMode)
		{
			mAngularVel = pAngularVel;
			mRobotVel = pRobotVel;
			mSwerveState = pSwerveState;
			mNudge = pNudge;
			mSpeedMode = pSpeedMode;
		}

		public SwerveMovement (
				double pAngularVel,
				Vector pRobotVel,
				SpeedMode pSpeedMode)
		{
			this (pAngularVel, pRobotVel, SwerveState.NORMAL, false, pSpeedMode);
		}

		public SwerveMovement (
				double pAngularVel,
				Vector pRobotVel,
				SwerveState pSwerveState,
				SpeedMode pSpeedMode)
		{
			this (pAngularVel, pRobotVel, pSwerveState, false, pSpeedMode);
		}

		public SwerveMovement()
		{
			this (0, new Vector(), SwerveState.DO_NOTHING, false, SpeedMode.INDIFFERENT);
		}
	}

	public static final Vector
	FORWARD = new Vector(1.0, 0),
	LEFT = new Vector(0, 1.0),
	BACK = new Vector(1.0, 0),
	RIGHT = new Vector (0, -1.0);

	public static final SwerveMovement
	WHEELS_FB = new SwerveMovement (0, FORWARD, SwerveMovement.SwerveState.STAY_STILL, SpeedMode.INDIFFERENT),
	WHEELS_LR = new SwerveMovement (0, LEFT, SwerveMovement.SwerveState.STAY_STILL, SpeedMode.INDIFFERENT),
	WHEELS_ANGLING = new SwerveMovement (1.0, new Vector(), SwerveMovement.SwerveState.STAY_STILL, SpeedMode.INDIFFERENT),
	DO_NOTHING = new SwerveMovement (0, new Vector(), SwerveMovement.SwerveState.DO_NOTHING, SpeedMode.INDIFFERENT);

	public void setPOV (boolean pPOV)
	{
		if (pPOV)
		{
			mPOVMode = DirectionMode.POV_FORWARD;
		}
		else
		{
			mPOVMode = DirectionMode.FIELD_RELATIVE;
		}
	}
	
	public boolean getPOV ()
	{
		return mPOVMode != DirectionMode.FIELD_RELATIVE;
	}
	
	public void setSlowStick (boolean pSlowStick)
	{
		mSlowStick = pSlowStick;
	}
	
	private void setGyroPIDGains (boolean isFast) {
		//set deadband
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			mGyroPID.setPID(
					DriveConstants.PID_Constants.Prototype.ANGLE_P, 
					DriveConstants.PID_Constants.Prototype.ANGLE_I,
					DriveConstants.PID_Constants.Prototype.ANGLE_D);

			mGyroPID.setOutputRange(
					-DriveConstants.PID_Constants.Prototype.ANGLE_MAX, 
					DriveConstants.PID_Constants.Prototype.ANGLE_MAX);

			mGyroPID.setIZone(DriveConstants.PID_Constants.Prototype.ANGLE_IZONE);
			mGyroPID.setDeadband(DriveConstants.PID_Constants.Prototype.ANGLE_DEADBAND);
		}
		else
		{
			if (isFast) {
				mGyroPID.setPID(
						DriveConstants.PID_Constants.Real.ANGLE_FAST_P, 
						DriveConstants.PID_Constants.Real.ANGLE_FAST_I,
						DriveConstants.PID_Constants.Real.ANGLE_FAST_D);

				mGyroPID.setOutputRange(
						-DriveConstants.PID_Constants.Real.ANGLE_FAST_MAX, 
						DriveConstants.PID_Constants.Real.ANGLE_FAST_MAX);

				mGyroPID.setIZone(DriveConstants.PID_Constants.Real.ANGLE_FAST_IZONE);
				mGyroPID.setDeadband(DriveConstants.PID_Constants.Real.ANGLE_DEADBAND);
			}
			else {
				mGyroPID.setPID(
						DriveConstants.PID_Constants.Real.ANGLE_SLOW_P, 
						DriveConstants.PID_Constants.Real.ANGLE_SLOW_I,
						DriveConstants.PID_Constants.Real.ANGLE_SLOW_D);

				mGyroPID.setOutputRange(
						-DriveConstants.PID_Constants.Real.ANGLE_SLOW_MAX, 
						DriveConstants.PID_Constants.Real.ANGLE_SLOW_MAX);

				mGyroPID.setIZone(DriveConstants.PID_Constants.Real.ANGLE_SLOW_IZONE);
				mGyroPID.setDeadband(DriveConstants.PID_Constants.Real.ANGLE_DEADBAND);
			}
		}		
	}

	private void setDriftCompensationPIDGains (boolean isFast) {
		//set deadband
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			mDriftCompensationPID.setPID(
					DriveConstants.PID_Constants.Prototype.DRIFT_COMP_TARGET_P, 
					DriveConstants.PID_Constants.Prototype.DRIFT_COMP_TARGET_I,
					0);

			mDriftCompensationPID.setOutputRange(
					-DriveConstants.PID_Constants.Prototype.DRIFT_COMP_MAX, 
					DriveConstants.PID_Constants.Prototype.DRIFT_COMP_MAX);
		}
		else
		{
			if (isFast) {
				mDriftCompensationPID.setPID(
						DriveConstants.PID_Constants.Real.DRIFT_COMP_FAST_TARGET_P, 
						DriveConstants.PID_Constants.Real.DRIFT_COMP_FAST_TARGET_I,
						0);

				mDriftCompensationPID.setOutputRange(
						-DriveConstants.PID_Constants.Real.DRIFT_COMP_FAST_TARGET_MAX, 
						DriveConstants.PID_Constants.Real.DRIFT_COMP_FAST_TARGET_MAX);
			}
			else {
				mDriftCompensationPID.setPID(
						DriveConstants.PID_Constants.Real.DRIFT_COMP_SLOW_TARGET_P, 
						DriveConstants.PID_Constants.Real.DRIFT_COMP_SLOW_TARGET_I,
						0);

				mDriftCompensationPID.setOutputRange(
						-DriveConstants.PID_Constants.Real.DRIFT_COMP_SLOW_TARGET_MAX, 
						DriveConstants.PID_Constants.Real.DRIFT_COMP_SLOW_TARGET_MAX);
			}
		}		
	}

	public boolean getAllWheelsInRange() {
		boolean allInRange = true;
		for (int i = 0; i < 4; i++) {
			if (!mWheels[i].isInRangeNudge())
			{
				allInRange = false;
			}
		}

		return allInRange;
	}

	private void setShiftMode (boolean isFast) {
		if (RunConstants.RUNNING_PNEUMATIC_SHIFTER) 
		{
			boolean prevIsFast = (mShifterSolenoid.get() == DriveConstants.FAST_SHIFT_DIR);
			if (prevIsFast != isFast)
			{
				SmartDashboard.putString ("Changing Shift Mode:", (prevIsFast ? "FAST" : "SLOW") + " --> " + (isFast ? "FAST" : "SLOW"));
				for (Wheel w : mWheels)
				{
					w.setSpeedMode(isFast);
				}
				mShifterSolenoid.set(isFast ? DriveConstants.FAST_SHIFT_DIR : DriveConstants.SLOW_SHIFT_DIR);
				setGyroPIDGains(isFast);
				setDriftCompensationPIDGains(isFast);
			}
		}
	}

	public void enactMovement (SwerveMovement swerveMovement)
	{
		
		long timeStartedEverything = System.currentTimeMillis();
		boolean allInRange = getAllWheelsInRange();
		SmartDashboard.putBoolean("All Wheels In Range:", allInRange);
		SmartDashboard.putBoolean("Drift Compensating", swerveMovement.mSwerveState == SwerveState.HOLD_FOR_DIRECTION);
		if (swerveMovement.mSwerveState == SwerveState.HOLD_FOR_DIRECTION)
		{
			SmartDashboard.putNumber("Drift Compensating Vel", swerveMovement.mAngularVel);
		}
		
		if (RunConstants.RUNNING_PNEUMATIC_SHIFTER)
		{
			SmartDashboard.putString("Shift Mode", mShifterSolenoid.get() == DriveConstants.FAST_SHIFT_DIR ? "FAST" : "SLOW");
		}
		boolean shouldDrive = true;
		
		long timeStartedCalculation = System.currentTimeMillis();
		
		switch (swerveMovement.mSwerveState)
		{
		case NORMAL:
			mSwerveDrive.calculate(swerveMovement.mAngularVel, swerveMovement.mRobotVel);
			break;
		case HOLD_FOR_DIRECTION:
			mSwerveDrive.calculateHoldDirection(swerveMovement.mAngularVel, swerveMovement.mRobotVel);
			break;
		case STAY_STILL:
			mSwerveDrive.calculate(swerveMovement.mAngularVel, swerveMovement.mRobotVel);
			shouldDrive = false;
			break;
		case DO_NOTHING:
			mSwerveDrive.calculate(0.0, new Vector());
			shouldDrive = false;
			break;
		}
		
		long timeEndedCalculation = System.currentTimeMillis();
		SmartDashboard.putNumber("Time To Calculate Swerve:", timeEndedCalculation - timeStartedCalculation);

		if (RunConstants.RUNNING_PNEUMATIC_SHIFTER)
		{
			switch (swerveMovement.mSpeedMode)
			{
			case FAST:
				setShiftMode(true);
				break;
			case SLOW:
				setShiftMode(false);
				break;
			case INDIFFERENT:
				break;
			}
		}

		if (swerveMovement.mNudge && !allInRange)
		{
			shouldDrive = false;
		}

		if (swerveMovement.mSwerveState == SwerveState.DO_NOTHING)
		{
			mDesiredRobotVel = new Vector();
			mDesiredAngularVel = 0;
			
			for (int i = 0; i < 4; i++)

			{
				mWheels[i].setSpeed(0.0);
			}
		}
		else if (shouldDrive)
		{
			mDesiredRobotVel = new Vector (swerveMovement.mRobotVel);
			mDesiredAngularVel = swerveMovement.mAngularVel;
			
			for (int i = 0; i < 4; i++)
			{
				mWheels[i].set(mSwerveDrive.getOutput(i));
			}
		}
		else
		{
			mDesiredRobotVel = new Vector();
			mDesiredAngularVel = 0;
			
			for (int i = 0; i < 4; i++)
			{
				mWheels[i].setAngle(mSwerveDrive.getOutput(i).getAngle());
				mWheels[i].setSpeed(0.0);
			}
		}
		
		long timeEndedEverything = System.currentTimeMillis();
		SmartDashboard.putNumber("Time Enact Swerve Movement:", timeEndedEverything - timeStartedEverything);
	}

	private SwerveMovement getDriveMovement()
	{	
		if (mPOVMode != DirectionMode.POV_FORWARD)
		{
			mSlowStick = false;
		}
		
		if (mController.getStartButton()) 
		{
			mSlowStick = false;
			mPOVMode = DirectionMode.FIELD_RELATIVE;
		}
		if (mController.getBackButton())
		{
			mPOVMode = DirectionMode.POV_FORWARD;
		}
		if (mSecondaryStick.getRawButton(ControlPorts.AUTOMATIC_CLIMB_PORT))
		{
			mPOVMode = DirectionMode.FIELD_RELATIVE; //DirectionMode.POV_BACKWARD;
		}
		if (!mSecondaryStick.getRawButton(ControlPorts.AUTOMATIC_CLIMB_PORT) && mPOVMode == DirectionMode.POV_BACKWARD)
		{
			mPOVMode = DirectionMode.FIELD_RELATIVE;
		}

		SmartDashboard.putBoolean ("POV", getPOV());
		SmartDashboard.putBoolean ("Slow Stick", mSlowStick);
		
		//retrieve states based on joystick
		AngularVelState angularState = getAngleState();
		RobotVelState velState = getRobotVelState();

		//only preangle if we are doing nothing else
		if (angularState != AngularVelState.NONE && velState == RobotVelState.PREANGLE) {
			velState = RobotVelState.NONE;
		}

		SpeedMode speedMode = getSpeedMode (angularState, velState);

		//initialize angulerVel & driveVec to 0
		double angularVel = 0;
		Vector driveVec = new Vector();

		//calculate the angular velocity based on our state
		switch (angularState) {
		case NUDGE:
			angularVel = nudgeTurn();
			break;
		case STICK:
			angularVel = angularVelStick();
			break;
		case TURN_TO_0:
			angularVel = getAngularPIDVel(0);
			break;
		case TURN_TO_90:
			angularVel = getAngularPIDVel(90);
			break;
		case TURN_TO_180:
			angularVel = getAngularPIDVel(180);
			break;
		case TURN_TO_270:
			angularVel = getAngularPIDVel(270);
			break;
		case TURN_TO_45:
			angularVel = getAngularPIDVel(45);
			break;
		case TURN_TO_135:
			angularVel = getAngularPIDVel(135);
			break;
		case TURN_TO_225:
			angularVel = getAngularPIDVel(225);
			break;
		case TURN_TO_315:
			angularVel = getAngularPIDVel(315);
			break;
		case TURN_TO_60:
			angularVel = getAngularPIDVel(60);
			break;
		case TURN_TO_300:
			angularVel = getAngularPIDVel(300);
			break;
		case TURN_TO_JOYSTICK_DIR:
			angularVel = getAngularPIDVel(getStickAngle(Hand.kRight));
			break;
		case NONE:
			angularVel = 0;
			break;
		}

		//calculate linear velocity based on current state
		switch (velState) {
		case NUDGE:
			driveVec = nudgeMove();
			break;
		case PREANGLE:
			switch (mPOVMode)
			{
			case FIELD_RELATIVE:
				driveVec = swerveDriveVectorFieldRelative(1.0);
				break;
			case POV_FORWARD:
				driveVec = swerveDriveVectorRobotRelative(1.0);
				break;
			case POV_BACKWARD:
				driveVec = swerveDriveVectorRobotRelativeBackward(1.0);
				break;
			}
			break;
		case STICK:
			double speed = getStickLinearSpeed();
			switch (mPOVMode)
			{
			case FIELD_RELATIVE:
				driveVec = swerveDriveVectorFieldRelative(speed);
				break;
			case POV_FORWARD:
				driveVec = swerveDriveVectorRobotRelative(speed);
				break;
			case POV_BACKWARD:
				driveVec = swerveDriveVectorRobotRelativeBackward(speed);
				break;
			}

			break;
		case NONE:
			driveVec = new Vector();
			break;
		}


		//logic to determine if we need wheels to be accurate
		boolean shouldNudge = false;
		//if(velState == RobotVelState.NUDGE || velState == RobotVelState.ALIGN_PEG || angularState == AngularVelState.ALIGN_PEG) shouldNudge = true;
		if (velState == RobotVelState.NUDGE)
		{
			shouldNudge = true;
		}
		else if  (angularState == AngularVelState.NUDGE && velState == RobotVelState.NONE)
		{
			shouldNudge = true;
		}
		else
		{
			shouldNudge = false;
		}

		SwerveMovement.SwerveState swerveState = SwerveState.NORMAL;

		boolean canDriftCompensate = RunConstants.DRIFT_COMPENSATION;
		//if turning recently, dont compensate
		if (mTimeStoppedTurning > 0 &&
				System.currentTimeMillis() - mTimeStoppedTurning < 
				DriveConstants.PID_Constants.TIME_AFTER_TURNING_ACTIVATE_MILLIS) { 
			canDriftCompensate = false;
		}

		if (velState == RobotVelState.PREANGLE)
		{
			swerveState = SwerveState.STAY_STILL;
		}

		//we aren't doing anything, sit still and keep the wheel angles the same
		else if (driveVec.getMagnitude() < DriveConstants.MIN_LINEAR_VEL && 
				Math.abs(angularVel) < DriveConstants.MIN_ANGULAR_VEL)
		{
			swerveState = SwerveState.DO_NOTHING;
		}

		//turn to angle while moving
		else if (driveVec.getMagnitude() >= DriveConstants.MIN_LINEAR_VEL &&
				angularState.isGyroPID)
		{
			angularVel *= 0.5;
		}

		//moving with or without rotation
		else {
			//perform drift compensation
			boolean shouldDriftCompensate = 
					canDriftCompensate &&
					angularState == AngularVelState.NONE;

			if (shouldDriftCompensate) {
				swerveState = SwerveState.HOLD_FOR_DIRECTION;
				angularVel = mDriftCompensationPID.get();
			}
			

			if (mSlowStick)
			{
				angularVel *= DriveConstants.SwerveSpeeds.MULT_SLOW_STICK_ANGLE;
				driveVec.scaleTotal(DriveConstants.SwerveSpeeds.MULT_SLOW_STICK_DRIVE);
			}
		}

		//disable the gyro PID if we didn't use it this iteration
		if (!angularState.isGyroPID) {
			mGyroPID.disable();
		}

		if (angularState != AngularVelState.NONE)
		{
			mTimeStoppedTurning = System.currentTimeMillis();
		}

		if (swerveState == SwerveState.HOLD_FOR_DIRECTION)
		{
			if (!mDriftCompensationPID.isEnable())
			{
				mDriftCompensationPID.reset();
				mDriftCompensationPID.enable();
			}
			
		}
		else
		{
			if (mDriftCompensationPID.isEnable())
			{
				mDriftCompensationPID.disable();
			}
			mDriftCompensationPID.setSetpoint(mRobotAngle.getAngleDegrees());
		}
		
		double robotDirAngleAdamDashboard = 0;
		switch (velState)
		{
		case NONE:
			switch (mPOVMode)
			{
			case FIELD_RELATIVE:
				robotDirAngleAdamDashboard = ResourceFunctions.putAngleInRange(-getAngleDegrees());
				break;
			case POV_FORWARD:
				robotDirAngleAdamDashboard = 0;
				break;
			case POV_BACKWARD:
				robotDirAngleAdamDashboard = 180;
				break;
			}
		case NUDGE:
			robotDirAngleAdamDashboard = driveVec.getAngle();
			break;
		case PREANGLE:
			robotDirAngleAdamDashboard = driveVec.getAngle();
			break;
		case STICK:
			robotDirAngleAdamDashboard = driveVec.getAngle();
			break;
		}
		SmartDashboard.putNumber("RobotDirection", robotDirAngleAdamDashboard);
		
		return new SwerveMovement(angularVel, driveVec, swerveState, shouldNudge, speedMode);
	}

	public void drive()
	{
		enactMovement(getDriveMovement());
	}

	public void driveTank()
	{
		double forwardVel = mController.getTriggerAxis(Hand.kRight) - mController.getTriggerAxis(Hand.kLeft);
		forwardVel *= 0.4;
		double angularVel = getStickAngle(Hand.kRight);
		angularVel *= 0.1;
		
		double leftSpeed = forwardVel - angularVel;
		double rightSpeed = forwardVel + angularVel;
		
		for (int i = 0; i < 4; i++)
		{
			mWheels[0].setAngle(0);
		}
		
		mWheels[0].setSpeed (rightSpeed);
		mWheels[1].setSpeed (leftSpeed);
		mWheels[2].setSpeed (leftSpeed);
		mWheels[3].setSpeed (rightSpeed);
		
		
	}
	
	private SpeedMode getSpeedMode (AngularVelState pAngularState, RobotVelState pVelState)
	{
		SpeedMode speedModeAng = SpeedMode.INDIFFERENT;
		SpeedMode speedModeVel = SpeedMode.INDIFFERENT;

		switch (pAngularState)
		{
		case NUDGE:
			speedModeAng = SpeedMode.SLOW;
			break;
		case STICK:
			speedModeAng = mSlowStick ? SpeedMode.SLOW : SpeedMode.FAST;
			break;
		default:
			speedModeAng = SpeedMode.INDIFFERENT;
			break;
		}
		
		switch (pVelState)
		{
		case NUDGE:
			speedModeVel = SpeedMode.SLOW;
			break;
		case STICK:
			speedModeVel = mSlowStick ? SpeedMode.SLOW : SpeedMode.FAST;
			break;
		default:
			speedModeVel = SpeedMode.INDIFFERENT;
			break;
		}
		
		SpeedMode finalSpeedMode = SpeedMode.INDIFFERENT;
		if (speedModeAng == SpeedMode.FAST || speedModeVel == SpeedMode.FAST)
		{
			finalSpeedMode = SpeedMode.FAST;
		}
		
		if (speedModeAng == SpeedMode.SLOW || speedModeVel == SpeedMode.SLOW)
		{
			finalSpeedMode = SpeedMode.SLOW;
		}
		
		return finalSpeedMode;
	}
	
	/**
	 * Determines angular velocity state,
	 * this state is then used to find the angular velocity of the swerve drive
	 * @return correct angular state
	 */
	private AngularVelState getAngleState() {
		AngularVelState angularState = AngularVelState.NONE;

		//		if (mAlignPegState == AlignState.ALIGN_ANGLE_1 || mAlignPegState == AlignState.ALIGN_ANGLE_2) {
		//			angularState = AngularVelState.ALIGN_PEG;
		//		}
		if (mController.getBumper(Hand.kLeft) || mController.getBumper(Hand.kRight)) {
			angularState = AngularVelState.NUDGE;
		}

		//pov buttons pressed, rotate to an angle
		else if (mController.getPOV() == 0) {
			angularState = AngularVelState.TURN_TO_0;
		}

		else if (mController.getPOV() == 90) {
			angularState = AngularVelState.TURN_TO_270;
		}

		else if (mController.getPOV() == 180) {
			angularState = AngularVelState.TURN_TO_180;
		}

		else if (mController.getPOV() == 270) {
			angularState = AngularVelState.TURN_TO_90;
		}

		else if (mSecondaryStick.getRawButton(ControlPorts.MIDDLE_STATION_BUTTON)) {
			angularState = AngularVelState.TURN_TO_0;
		}

		else if (mSecondaryStick.getRawButton(ControlPorts.RIGHT_STATION_BUTTON)) {
			angularState = AngularVelState.TURN_TO_60;
		}

		else if (mSecondaryStick.getRawButton(ControlPorts.LEFT_STATION_BUTTON)) {
			angularState = AngularVelState.TURN_TO_300;
		}

		//REMOVE THIS LATER, FOR ANGULAR PID TESTING
		//		else if (getStickMag(Hand.kRight) > 0.4 && mController.getAButton()) {
		//			angularState = AngularVelState.TURN_TO_JOYSTICK_DIR;
		//		}

		//if the right joystick is pushed beyond the minimum angular vel, 
		else if (Math.abs(angularVelStick()) > DriveConstants.MIN_ANGULAR_VEL_STICK) {
			angularState = AngularVelState.STICK;
		}


		return angularState;
	}

	/**
	 * Determines linear velocity state,
	 * this state is then used to find the correct linear velocity of the swerve drive
	 * @return correct linear velocity state
	 */
	private RobotVelState getRobotVelState() {
		RobotVelState velState = RobotVelState.NONE;

		//		if (mAlignPegState == AlignState.ALIGN_LR) {
		//			velState = RobotVelState.ALIGN_PEG;
		//		}
		if(false
				/*mController.getAButton() ||*/
				/*mController.getBButton() ||*/
				/*mController.getXButton() ||*/
				/*mController.getYButton()*/) {
			velState = RobotVelState.NUDGE;
			//velState = RobotVelState.NONE; //FOR ANGULAR PID TESTING
		}

		else if ((mController.getTriggerAxis(Hand.kRight) > DriveConstants.MIN_LINEAR_VEL || 
				mController.getTriggerAxis(Hand.kLeft)  > DriveConstants.MIN_LINEAR_VEL) &&
				getStickMag(Hand.kLeft) > DriveConstants.MIN_DIRECTION_MAG) {
			velState = RobotVelState.STICK;
		}

		/*
		 * Diferent control system based on left stick field relative & right stick robot relative
		else if((mController.getTriggerAxis(Hand.kRight) > DriveConstants.MIN_LINEAR_VEL || 
				 mController.getTriggerAxis(Hand.kLeft)  > DriveConstants.MIN_LINEAR_VEL) &&
				getStickMag(Hand.kRight) > DriveConstants.MIN_DIRECTION_MAG) {
			return RobotVelState.STICK_ROBOT_RELATIVE;
		}
		 */

		else if(getStickMag(Hand.kLeft) > 0.3) { //this makes it easier to choose a direction and THEN start driving
			velState = RobotVelState.PREANGLE;
		}


		return velState;			
	}


	/**
	 * Gets the angle of Xbox controller joystick 
	 * @param h Joystick to get the angle for
	 * @return Angle of the stick in degrees, with 0 degrees
	 * pointing directly up on the controller
	 */
	public double getStickAngle(Hand h) {
		double angle = Math.toDegrees(Math.atan2(-mController.getY(h), mController.getX(h)));
		angle -= 90;
		return ResourceFunctions.putAngleInRange(angle);
	}

	/**
	 * Gets magnitude of Xbox controller joystick 
	 * @param h Joystick to get the magnitude for
	 * @return Magnitude of the depression; 0 is not used, 1 is fully depressed
	 */
	public double getStickMag(Hand h) {
		Vector v = new Vector(mController.getX(h), mController.getY(h));
		return v.getMagnitude();
	}

	/**
	 * Get the direction vector for nudge driving using the letter buttons
	 * @return correct direction vector
	 */
	private Vector nudgeMove()
	{
		Vector driveVec = new Vector();
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			if(mController.getYButton())
			{
				driveVec = Vector.createPolar(0, 
						DriveConstants.SwerveSpeeds.Prototype.NUDGE_MOVE_SPEED);
			}
			else if(mController.getXButton()) 
			{
				driveVec = Vector.createPolar(90, 
						DriveConstants.SwerveSpeeds.Prototype.NUDGE_MOVE_SPEED);
			}
			else if(mController.getAButton()) 
			{
				driveVec = Vector.createPolar(180, 
						DriveConstants.SwerveSpeeds.Prototype.NUDGE_MOVE_SPEED);
			}
			else if(mController.getBButton()) 
			{
				driveVec = Vector.createPolar(270, 
						DriveConstants.SwerveSpeeds.Prototype.NUDGE_MOVE_SPEED);
			}
		}
		else
		{
			if(mController.getYButton()) 
			{
				driveVec = Vector.createPolar(0, 
						DriveConstants.SwerveSpeeds.Real.NUDGE_MOVE_SPEED);
			}
			else if(mController.getXButton())
			{
				driveVec = Vector.createPolar(90, 
						DriveConstants.SwerveSpeeds.Real.NUDGE_MOVE_SPEED);
			}
			else if(mController.getAButton())
			{
				driveVec = Vector.createPolar(180, 
						DriveConstants.SwerveSpeeds.Real.NUDGE_MOVE_SPEED);
			}
			else if(mController.getBButton()) 
			{
				driveVec = Vector.createPolar(270, 
						DriveConstants.SwerveSpeeds.Real.NUDGE_MOVE_SPEED);
			}
		}

		return driveVec;
	}

	/**
	 * Angular velocity using nudge bumpers
	 * @return correct angular velocity
	 */
	private double nudgeTurn() 
	{
		double speed = 0;
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			if(mController.getBumper(Hand.kLeft))
			{
				speed = DriveConstants.SwerveSpeeds.Prototype.NUDGE_TURN_SPEED;
			}
			else if(mController.getBumper(Hand.kRight))
			{
				speed = -DriveConstants.SwerveSpeeds.Prototype.NUDGE_TURN_SPEED;
			}
		}
		else
		{
			if(mController.getBumper(Hand.kLeft))
			{
				speed = DriveConstants.SwerveSpeeds.Real.NUDGE_TURN_SPEED;
			}
			else if(mController.getBumper(Hand.kRight))
			{
				speed = -DriveConstants.SwerveSpeeds.Real.NUDGE_TURN_SPEED;
			}
		}

		return speed;
	}
	
	private double getStickLinearSpeed ()
	{
		double speed = mController.getTriggerAxis(Hand.kRight);
		speed *= Math.abs(speed); //quadratic control, finer control of lower speeds

		speed *= RunConstants.IS_PROTOTYPE_BOT ? 
				DriveConstants.SwerveSpeeds.Prototype.SPEED_MULT : 
					DriveConstants.SwerveSpeeds.Real.SPEED_MULT; //lowers the maximum speed
		
		return speed;
	}

	/**
	 * Calculates robot movement vector assuming field relative control with left xbox joystick
	 * good for controlling the robot while looking at it
	 * @return robot relative direction vector to move in a certain direction on the field
	 */
	private Vector swerveDriveVectorFieldRelative(double pSpeed) {
		double robotAngle = mRobotAngle.getAngleDegrees();
		double driveAngle = getStickAngle(Hand.kLeft);
		driveAngle -= robotAngle; //make it field relative
		driveAngle = ResourceFunctions.putAngleInRange(driveAngle);

		return Vector.createPolar(driveAngle, pSpeed);
	}

	/**
	 * calculates robot movement vector assuming robot relative control with left xbox joystick
	 * good for POV control through camera
	 * @return robot relative direction vector assuming robot relative control
	 */
	private Vector swerveDriveVectorRobotRelative(double pSpeed) {
		double driveAngle = getStickAngle(Hand.kLeft);
		
		driveAngle = ResourceFunctions.putAngleInRange(driveAngle);
		
		return Vector.createPolar(driveAngle, pSpeed);
	}
	
	private Vector swerveDriveVectorRobotRelativeBackward(double pSpeed) {
		double driveAngle = getStickAngle(Hand.kLeft);

		driveAngle += 180;
		driveAngle = ResourceFunctions.putAngleInRange(driveAngle);
		
		return Vector.createPolar(driveAngle, pSpeed);
	}

	/**
	 * Angular velocity calculated with the right joystick
	 * @return angular velocity for swerve drive
	 */
	private double angularVelStick() {
		double angularVel = mController.getX(Hand.kRight) * Math.abs(mController.getX(Hand.kRight));

		angularVel *= RunConstants.IS_PROTOTYPE_BOT ? 
				DriveConstants.SwerveSpeeds.Prototype.ANGULAR_SPEED_MULT : 
					DriveConstants.SwerveSpeeds.Real.ANGULAR_SPEED_MULT; //lowers the maximum speed

		angularVel = -angularVel; //correct the sign for clockwise/counter-clockwise
		return angularVel; //quadratic control for finer movements
	}

	/**
	 * Calculate angular velocity to turn to a certain angle
	 * @param setpointAngle angle to turn to
	 * @return angular velocity required to turn to the angle
	 */
	public double getAngularPIDVel (double setpointAngle) {
		mGyroPID.setSetpoint(setpointAngle);
		
		if (!mGyroPID.isEnable())
		{
			mGyroPID.enable();
		}
		
		
		//makes sure that the gyro PID has updated before we use it
		while (mGyroPID_Output.getVal() > Configurables.ERROR_MIN) {
			Timer.delay(0.005); //this is ugly, should remove; however it shoud happen very rarey
		}

		double vel = mGyroPID_Output.getVal();
		SmartDashboard.putNumber("Angular PID Vel", vel);

		SmartDashboard.putNumber ("Gyro PID Setpoint:", mGyroPID.getSetpoint());
		SmartDashboard.putNumber ("Gyro PID Output:", vel);
		SmartDashboard.putNumber("Gyro PID P:", mGyroPID.getP());

		return vel;
	}

	public double getDriftCompensationAngularVel ()
	{
		return mDriftCompensationPID.get();
	}

	public void resetDriftCompensationPID()
	{
		mDriftCompensationPID.setSetpoint(mRobotAngle.getAngleDegrees());
		mDriftCompensationPID.reset();
		mDriftCompensationPID.enable();
	}

	public PersistentlyInRange getGyroInRange(long pMinTime, double pTolerance)
	{
		return new PersistentlyInRange(mGyroPID, pMinTime, pTolerance);
	}

	public double getAngleDegrees()
	{
		return mRobotAngle.getAngleDegrees();
	}
	
	public Vector getDesiredRobotVel()
	{
		return mDesiredRobotVel;
	}
	
	public double getDesiredAngularVel()
	{
		return mDesiredAngularVel;
	}
}


/*
private double getDriftAngularVel (double pDriveMagnitude, double pGyroAngularVel)
	{
		double driftCompensateAngularVel = 0;
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			if (pDriveMagnitude < 
					DriveConstants.PID_Constants.Prototype.DRIFT_COMPENSATION_ZONE_1) 
			{
				driftCompensateAngularVel = 
						-DriveConstants.PID_Constants.Prototype.DRIFT_COMPENSATION_P1 * pGyroAngularVel;
			}
			else if (pDriveMagnitude < 
					DriveConstants.PID_Constants.Prototype.DRIFT_COMPENSATION_ZONE_2) 
			{
				driftCompensateAngularVel = 
						-DriveConstants.PID_Constants.Prototype.DRIFT_COMPENSATION_P2 * pGyroAngularVel;
			}
			else {
				driftCompensateAngularVel = 
						-DriveConstants.PID_Constants.Prototype.DRIFT_COMPENSATION_P3 * pGyroAngularVel;
			}
		}
		else
		{
			if (pDriveMagnitude < 
					DriveConstants.PID_Constants.Real.DRIFT_COMPENSATION_ZONE_1) 
			{
				driftCompensateAngularVel = 
						-DriveConstants.PID_Constants.Real.DRIFT_COMPENSATION_P1 * pGyroAngularVel;
			}
			else if (pDriveMagnitude < 
					DriveConstants.PID_Constants.Real.DRIFT_COMPENSATION_ZONE_2) 
			{
				driftCompensateAngularVel = 
						-DriveConstants.PID_Constants.Real.DRIFT_COMPENSATION_P2 * pGyroAngularVel;
			}
			else {
				driftCompensateAngularVel = 
						-DriveConstants.PID_Constants.Real.DRIFT_COMPENSATION_P3 * pGyroAngularVel;
			}
		}

		return driftCompensateAngularVel;
	}


public void driveOld() {

	if (mController.getStartButton()) 
	{
		mPOVMode = false;
	}
	if (mController.getBackButton())
	{
		mPOVMode = true;
	}

	SmartDashboard.putBoolean ("POV", mPOVMode);

	double curAngle = mRobotAngle.getAngleDegrees();	

	//retrieve states based on joystick
	AngularVelState angularState = getAngleState();
	RobotVelState velState = getRobotVelState();

	//only preangle if we are doing nothing else
	if (angularState != AngularVelState.NONE && velState == RobotVelState.PREANGLE) {
		velState = RobotVelState.NONE;
	}

	if (RunConstants.RUNNING_PNEUMATIC_SHIFTER)
	{
		boolean prevIsFast = (mShifterSolenoid.get() == DriveConstants.FAST_SHIFT_DIR);
		boolean newIsFast = prevIsFast;
		if (velState.speedMode == SpeedMode.SLOW || angularState.speedMode == SpeedMode.SLOW) {
			newIsFast = false;
		}
		else if (velState.speedMode == SpeedMode.FAST || angularState.speedMode == SpeedMode.FAST) {
			newIsFast = true;
		}
		setShiftMode(newIsFast);
	}

	boolean driftCompensating = false;
	//initialize angulerVel & driveVec to 0
	double angularVel = 0;
	Vector driveVec = new Vector();

	//calculate the angular velocity based on our state
	switch (angularState) {
	case NUDGE:
		angularVel = nudgeTurn();
		break;
	case STICK:
		angularVel = angularVelStick();
		break;
	case TURN_TO_0:
		angularVel = getAngularPIDVel(0);
		break;
	case TURN_TO_90:
		angularVel = getAngularPIDVel(90);
		break;
	case TURN_TO_180:
		angularVel = getAngularPIDVel(180);
		break;
	case TURN_TO_270:
		angularVel = getAngularPIDVel(270);
		break;
	case TURN_TO_45:
		angularVel = getAngularPIDVel(45);
		break;
	case TURN_TO_135:
		angularVel = getAngularPIDVel(135);
		break;
	case TURN_TO_225:
		angularVel = getAngularPIDVel(225);
		break;
	case TURN_TO_315:
		angularVel = getAngularPIDVel(315);
		break;
	case TURN_TO_60:
		angularVel = getAngularPIDVel(60);
		break;
	case TURN_TO_300:
		angularVel = getAngularPIDVel(300);
		break;
	case TURN_TO_JOYSTICK_DIR:
		angularVel = getAngularPIDVel(getStickAngle(Hand.kRight));
		break;
	case NONE:
		angularVel = 0;
		break;
	}

	//calculate linear velocity based on current state
	switch (velState) {
	case NUDGE:
		driveVec = nudgeMove();
		break;
	case STICK:
		driveVec = mPOVMode ? swerveDriveVectorRobotRelative() : swerveDriveVectorFieldRelative();
		break;
	default:
		driveVec = new Vector(); //0 vector if doing nothing
	}


	//logic to determine if we need wheels to be accurate
	boolean shouldNudge = false;
	//if(velState == RobotVelState.NUDGE || velState == RobotVelState.ALIGN_PEG || angularState == AngularVelState.ALIGN_PEG) shouldNudge = true;
	if (velState == RobotVelState.NUDGE) shouldNudge = true;
	else if  (angularState == AngularVelState.NUDGE && velState == RobotVelState.NONE) shouldNudge = true;
	else shouldNudge = false;

	boolean shouldDriveWhenNudging = getAllWheelsInRange();

	boolean canDriftCompensate = RunConstants.DRIFT_COMPENSATION;
	if (Math.abs(mRobotAngle.getAngularVelocity()) >
			DriveConstants.PID_Constants.MAX_ANGULAR_VELOCITY_COMPENSATE)
	{
		canDriftCompensate = false;
	}

	//if turning recently, dont compensate
	if (mTimeStoppedTurning > 0 &&
			System.currentTimeMillis() - mTimeStoppedTurning < 
			DriveConstants.PID_Constants.TIME_AFTER_TURNING_ACTIVATE_MILLIS) { 
		canDriftCompensate = false;
	}

	//manually set the wheels to the right angle, and don't drive them; for choosing angle & then driving
	if(velState == RobotVelState.PREANGLE) {
		zeroMotors();
		double angle = getStickAngle(Hand.kLeft);
		if (!mPOVMode)
		{
			angle -= curAngle;
		}
		for(Wheel w : mWheels) {
			w.set(angle, 0);
		}
	}

	//we aren't doing anything, sit still and keep the wheel angles the same
	else if (driveVec.getMagnitude() < DriveConstants.MIN_LINEAR_VEL && 
			Math.abs(angularVel) < DriveConstants.MIN_ANGULAR_VEL &&
			!angularState.isGyroPID) 
	{
		zeroMotors();
	}

	//turn to angle while still, cant fit into the above because we need to be able to rotate slowly
	else if (driveVec.getMagnitude() < DriveConstants.MIN_LINEAR_VEL && 
			angularState.isGyroPID)
	{
		//get wheel angles
		mSwerveDrive.calculate(1.0, new Vector());

		for (int i = 0; i < 4; i++)
		{
			mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), angularVel);
		}
	}

	//turn to angle while moving
	else if (driveVec.getMagnitude() >= DriveConstants.MIN_LINEAR_VEL &&
			angularState.isGyroPID)
	{

		mSwerveDrive.calculate(angularVel * 0.5, driveVec);

		//SmartDashboard.putNumber("Gyro PID Angular" + i, mSwerveDrive.getOutput(i).getMagnitude());

		for(int i = 0; i < 4; i++) {
			if (shouldNudge && !shouldDriveWhenNudging) {
				mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), 0);
			}
			else {
				mWheels[i].set(mSwerveDrive.getOutput(i));
			}


		}
	}

	//moving with or without rotation
	else {
		//perform drift compensation
		boolean shouldDriftCompensate = 
				canDriftCompensate &&
				angularState == AngularVelState.NONE;

		if (shouldDriftCompensate) {
			driftCompensating = true;

			double mag = driveVec.getMagnitude();
			double gyroAngularVel = mRobotAngle.getAngularVelocity();

			double driftCompensateAngularVel = 0;
			if (DriveConstants.USING_DRIFT_COMP_PID)
			{
				driftCompensateAngularVel = mDriftCompensationPID.get();
			}
			else
			{
				driftCompensateAngularVel = getDriftAngularVel(mag, gyroAngularVel);
			}

			mSwerveDrive.calculateHoldDirection(driftCompensateAngularVel, driveVec);
			SmartDashboard.putNumber("Drift Compensation Angular Vel:", driftCompensateAngularVel);
			SmartDashboard.putNumber("Drift Compensation Angle Off:", mDriftCompensationPID.GetError());
		}

		//we are doing something active: calculate wheel vectors w/ swerve drive math
		else {
			mSwerveDrive.calculate(angularVel, driveVec);
		}

		//set swerve wheels
		for(int i = 0; i < 4; i++) {
			if (shouldNudge && !shouldDriveWhenNudging) {
				mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), 0);
			}
			else {
				mWheels[i].set(mSwerveDrive.getOutput(i));
			}
		}
	}

	//disable the gyro PID if we didn't use it this iteration
	if (!angularState.isGyroPID) {
		mGyroPID.disable();
		//write an error value so we know not to take the last output when reactivated
		mGyroPID_Output.pidWrite(Configurables.ERROR_MIN + 10);
	}

	if (angularState != AngularVelState.NONE)
	{
		mTimeStoppedTurning = System.currentTimeMillis();
	}

	SmartDashboard.putBoolean("Drift Compensating", driftCompensating);
	if (driftCompensating)
	{
		mDriftCompensationPID.enable();
	}
	else
	{
		mDriftCompensationPID.disable();
		mDriftCompensationPID.setSetpoint(mRobotAngle.getAngleDegrees());
	}
}

 */