package robotcode.macros;

import constants.DriveConstants;
import constants.RunConstants;
import constants.ZedAlignmentConstants;
import edu.wpi.first.wpilibj.LocalPIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.Vector;
import robotcode.PoseEstimator;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;
import robotcode.pid.GenericPIDOutput;
import robotcode.pid.PersistentlyInRange;

public class AlignPegZedHybrid extends DriveMacro {

	private PoseEstimator mJetsonTX1UDP;
	private PIDSource mLRInput, mAngleInput;
	private GenericPIDOutput mLROutput, mAngleOutput;
	private LocalPIDController mLRPID, mAnglePID_Moving, mAnglePID_Initial;
	private AlignmentState mAlignState;
	private PersistentlyInRange mLRInRange, mAngleInRange;
	private XboxController mXboxController;
	
	enum AlignmentState
	{
		SEARCHING,
		ALIGN_ANGLE_APPROX,
		ALIGN_APPROX,
		FORWARD_MANUAL
	}
	
	public AlignPegZedHybrid(DriveTrain pDriveTrain, PoseEstimator pJetsonTX1UDP, XboxController pXboxController) {
		super(pDriveTrain);
		mJetsonTX1UDP = pJetsonTX1UDP;
		mXboxController = pXboxController;
		// TODO Auto-generated constructor stub
	}
	
	public void start()
	{
		/*mFBInput = new PIDSource() {
			public double pidGet() {
				return mJetsonTX1UDP.getFB();
			}

			public PIDSourceType getPIDSourceType() {return null;}
			public void setPIDSourceType(PIDSourceType pidSource) {}
		};*/

		mLRInput = new PIDSource() {
			public double pidGet() {
				return mJetsonTX1UDP.getLR();
			}

			public PIDSourceType getPIDSourceType() {return null;}
			public void setPIDSourceType(PIDSourceType pidSource) {}
		};

		mAngleInput = new PIDSource() {
			public double pidGet() {
				return mJetsonTX1UDP.getAngle();
			}

			public PIDSourceType getPIDSourceType() {return null;}
			public void setPIDSourceType(PIDSourceType pidSource) {}
		};
		
		mAlignState = AlignmentState.SEARCHING;
		
		mLROutput = new GenericPIDOutput();
		mAngleOutput = new GenericPIDOutput();
		mLRPID = new LocalPIDController(
				ZedAlignmentConstants.LR_P,
				ZedAlignmentConstants.LR_I, 
				ZedAlignmentConstants.LR_D, 
				mLRInput, mLROutput);
		mLRPID.setOutputRange(-ZedAlignmentConstants.LR_MAX, ZedAlignmentConstants.LR_MAX);
		mLRPID.setSetpoint (ZedAlignmentConstants.LR_SETPOINT);
		mLRPID.enable();
		
		mAnglePID_Moving = new LocalPIDController(
				ZedAlignmentConstants.ANGLE_P,
				ZedAlignmentConstants.ANGLE_I, 
				ZedAlignmentConstants.ANGLE_D, 
				mAngleInput, mAngleOutput);
		mAnglePID_Moving.setOutputRange(-ZedAlignmentConstants.ANGLE_MAX, ZedAlignmentConstants.ANGLE_MAX);
		mAnglePID_Moving.setInputRange(-180.0, 180.0);
		mAnglePID_Moving.setContinuous(true);
		mAnglePID_Moving.setSetpoint (ZedAlignmentConstants.ANGLE_SETPOINT);
		mAnglePID_Moving.enable();
		
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			mAnglePID_Initial = new LocalPIDController(
					DriveConstants.PID_Constants.Prototype.ANGLE_P,
					DriveConstants.PID_Constants.Prototype.ANGLE_I,
					DriveConstants.PID_Constants.Prototype.ANGLE_D,
					mAngleInput, mAngleOutput);
			mAnglePID_Initial.setOutputRange(
					-DriveConstants.PID_Constants.Prototype.ANGLE_MAX, 
					DriveConstants.PID_Constants.Prototype.ANGLE_MAX);
			
		}
		else
		{
			mAnglePID_Initial = new LocalPIDController(
					DriveConstants.PID_Constants.Real.ANGLE_SLOW_P,
					DriveConstants.PID_Constants.Real.ANGLE_SLOW_I,
					DriveConstants.PID_Constants.Real.ANGLE_SLOW_D,
					mAngleInput, mAngleOutput);
			mAnglePID_Initial.setOutputRange(
					-DriveConstants.PID_Constants.Real.ANGLE_SLOW_MAX, 
					DriveConstants.PID_Constants.Real.ANGLE_SLOW_MAX);
		}
		
		mAnglePID_Initial.setInputRange(-180.0, 180.0);
		mAnglePID_Initial.setContinuous(true);
		mAnglePID_Initial.setSetpoint (ZedAlignmentConstants.ANGLE_SETPOINT);
		mAnglePID_Initial.enable();
		
		mLRInRange = new PersistentlyInRange(mLRPID, 200, ZedAlignmentConstants.LR_INITIAL_TOLERANCE);
		mAngleInRange = new PersistentlyInRange(mAnglePID_Initial, 200, ZedAlignmentConstants.ANGLE_INITIAL_TOLERANCE);
	}

	protected SwerveMovement getMovement()
	{
		handleStates();
		SwerveMovement finalMovement = new SwerveMovement();
		switch (mAlignState)
		{
		case SEARCHING:
			finalMovement = DriveTrain.WHEELS_LR;
			break;
		case ALIGN_ANGLE_APPROX:
			finalMovement = getMovementAlignAngle();
			break;
		case ALIGN_APPROX:
			finalMovement = getMovementAlignApprox();
			break;
		case FORWARD_MANUAL:
			finalMovement = getMovementGoingForward();
			break;
		}
		
		return finalMovement;
	}
	
	private SwerveMovement getMovementAlignAngle ()
	{
		double angularVel = mAnglePID_Initial.get();
		return new SwerveMovement (angularVel, new Vector(), SwerveState.NORMAL, false, SpeedMode.SLOW);
	}
	
	private SwerveMovement getMovementAlignApprox() 
	{
		SmartDashboard.putNumber("PEG LR:", mLRInput.pidGet());
		double lrVel = mLRPID.get();
		double angularVel = mAnglePID_Moving.get();
		SmartDashboard.putNumber("OUTPUT LR:", lrVel);
		
		double moveDir = 90 - mJetsonTX1UDP.getAngle();
		
		if (Math.abs (lrVel) < 0.03)
		{
			return new SwerveMovement(0, Vector.createPolar(moveDir, 1.0), SwerveState.STAY_STILL, false, SpeedMode.SLOW);
		}
		else
		{
			return new SwerveMovement(angularVel, Vector.createPolar(moveDir, lrVel), SwerveState.HOLD_FOR_DIRECTION, false, SpeedMode.SLOW);
		}
	}
	
	private SwerveMovement getMovementGoingForward()
	{
		SmartDashboard.putNumber("PEG LR:", mLRInput.pidGet());
		double lrVel = - ZedAlignmentConstants.HUMAN_MULTIPLIER * mXboxController.getX(Hand.kLeft);
		double angularVel = mAnglePID_Moving.get();
		SmartDashboard.putNumber("OUTPUT LR:", lrVel);
		
		double lrDir = 90 - mJetsonTX1UDP.getAngle();
		double fbDir = -mJetsonTX1UDP.getAngle();
		
		Vector lrComponent = Vector.createPolar(lrDir, lrVel);
		Vector forwardComponent = Vector.createPolar(fbDir, ZedAlignmentConstants.FORWARD_SPEED);
		Vector totalVel = Vector.add (lrComponent, forwardComponent);
		return new SwerveMovement(angularVel, totalVel, SwerveState.HOLD_FOR_DIRECTION, false, SpeedMode.SLOW);
	}
	
	private void handleStates()
	{
		SmartDashboard.putString("Alignment State:", mAlignState.toString());
		SmartDashboard.putBoolean("Alignment Sees Valid:", mJetsonTX1UDP.getValid());
		switch (mAlignState)
		{
		case SEARCHING:
			if (mJetsonTX1UDP.getValid())
			{
				mAlignState = AlignmentState.ALIGN_ANGLE_APPROX;
			}
			break;
		case ALIGN_ANGLE_APPROX:
			if (!mJetsonTX1UDP.getValid())
			{
				mAlignState = AlignmentState.SEARCHING;
			}
			else if (mAngleInRange.inRangePersistently())
			{
				mAlignState = AlignmentState.ALIGN_APPROX;
			}
			break;
		case ALIGN_APPROX:
			if (!mJetsonTX1UDP.getValid())
			{
				mAlignState = AlignmentState.SEARCHING;
			}
			else if (mLRInRange.inRangePersistently())
			{
				mAlignState = AlignmentState.FORWARD_MANUAL;
			}
			break;
		case FORWARD_MANUAL:
			break;
		}
		
	}
	

	public boolean isComplete() {
		return false;
	}

}
