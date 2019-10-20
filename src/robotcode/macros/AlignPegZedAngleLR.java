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
import resource.ResourceFunctions;
import resource.Vector;
import robotcode.PoseEstimator;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;
import robotcode.pid.GenericPIDOutput;
import robotcode.pid.PersistentlyInRange;

public class AlignPegZedAngleLR extends DriveMacro {

	private PoseEstimator mPoseEstimator;
	private PIDSource mLRInput, mAngleInput;
	private GenericPIDOutput mLROutput, mAngleOutput;
	private LocalPIDController mLRPID, mAnglePID_Moving, mAnglePID_Initial;
	private AlignmentState mAlignState;
	private PersistentlyInRange mLRInRange, mAngleInRange;
	
	enum AlignmentState
	{
		SEARCHING,
		ALIGN_ANGLE_APPROX,
		ALIGN_LR_APPROX,
		COMPLETE
	}
	
	public AlignPegZedAngleLR(
			DriveTrain pDriveTrain, 
			PoseEstimator pPoseEstimator) {
		
		super(pDriveTrain);
		mPoseEstimator = pPoseEstimator;
	}
	
	public void start()
	{	

		mLRInput = new PIDSource() {
			public double pidGet() {
				return mPoseEstimator.getLR();
			}

			public PIDSourceType getPIDSourceType() {return null;}
			public void setPIDSourceType(PIDSourceType pidSource) {}
		};

		mAngleInput = new PIDSource() {
			public double pidGet() {
				return ResourceFunctions.putAngleInRange(mPoseEstimator.getAngle());
			}

			public PIDSourceType getPIDSourceType() {return null;}
			public void setPIDSourceType(PIDSourceType pidSource) {}
		};
		
		mAlignState = AlignmentState.SEARCHING;
		
		mLROutput = new GenericPIDOutput();
		mAngleOutput = new GenericPIDOutput();
		mLRPID = new LocalPIDController(
				0.05, 0.0, 0.0,
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
		
		mAnglePID_Initial.setInputRange(0, 360);
		mAnglePID_Initial.setContinuous(true);
		mAnglePID_Initial.setSetpoint (ZedAlignmentConstants.ANGLE_SETPOINT);
		mAnglePID_Initial.enable();
		
		mLRInRange = new PersistentlyInRange(mLRPID, -1, 1.0);
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
		case ALIGN_LR_APPROX:
			finalMovement = getMovementAlignApprox();
			break;
		case COMPLETE:
			finalMovement = DriveTrain.WHEELS_FB;
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
		
		double moveDir = 90 - mPoseEstimator.getAngle();
		
		if (Math.abs (lrVel) < 0.03)
		{
			return new SwerveMovement(0, Vector.createPolar(moveDir, 1.0), SwerveState.STAY_STILL, false, SpeedMode.SLOW);
		}
		else
		{
			return new SwerveMovement(angularVel, Vector.createPolar(moveDir, lrVel), SwerveState.HOLD_FOR_DIRECTION, false, SpeedMode.SLOW);
		}
	}

	private void handleStates()
	{
		SmartDashboard.putString("Alignment State:", mAlignState.toString());
		SmartDashboard.putBoolean("Alignment Sees Valid:", mPoseEstimator.getValid());
		switch (mAlignState)
		{
		case SEARCHING:
			if (mPoseEstimator.getValid())
			{
				mAlignState = AlignmentState.ALIGN_ANGLE_APPROX;
			}
			break;
		case ALIGN_ANGLE_APPROX:
			if (!mPoseEstimator.getValid())
			{
				mAlignState = AlignmentState.SEARCHING;
			}
			else if (mAngleInRange.inRangePersistently())
			{
				mAlignState = AlignmentState.ALIGN_LR_APPROX;
			}
			break;
		case ALIGN_LR_APPROX:
			if (!mPoseEstimator.getValid())
			{
				mAlignState = AlignmentState.SEARCHING;
			}
			else if (mLRInRange.inRangePersistently())
			{
				mAlignState = AlignmentState.COMPLETE;
			}
			break;
		case COMPLETE:
			break;
		}
		
	}
	

	public boolean isComplete() {
		return mAlignState == AlignmentState.COMPLETE;
	}

}
