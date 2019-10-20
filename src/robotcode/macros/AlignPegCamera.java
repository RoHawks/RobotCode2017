package robotcode.macros;

import constants.CamAlignerConstants;

import constants.RunConstants;
import edu.wpi.first.wpilibj.LocalPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.Vector;
import robotcode.JetsonData;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;
import robotcode.pid.GenericPIDInput;
import robotcode.pid.GenericPIDOutput;
import robotcode.pid.PersistentlyInRange;

//this is the one to change
public class AlignPegCamera extends DriveMacro
{
	private AlignState mAlignState;
	private LocalPIDController mLR_PID;
	private GenericPIDOutput mLR_PIDOutput;
	private GenericPIDInput mLR_PIDInput;

	private PersistentlyInRange mLRInRange;

	private double 
	LR_TOLERANCE, 
	FORWARD_VEL_INITIAL;

	public enum AlignState {
		NONE,
		SEARCHING,
		FORWARD_INITIAL,
		PREPARE_LR,
		ALIGN_LR_PERFECT,
		COMPLETE;
	}

	public AlignPegCamera(DriveTrain pDriveTrain) {
		super(pDriveTrain);
	}

	public void start()
	{
		super.start();
		mDriveTrain.resetDriftCompensationPID();

		mAlignState = AlignState.NONE;

		//initialize alignment PIDs
		mLR_PIDOutput = new GenericPIDOutput();
		mLR_PIDInput = new GenericPIDInput();

		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			mLR_PID = new LocalPIDController(
					CamAlignerConstants.PID_Constants.Prototype.LR_P,
					CamAlignerConstants.PID_Constants.Prototype.LR_I,
					CamAlignerConstants.PID_Constants.Prototype.LR_D,
					mLR_PIDInput, mLR_PIDOutput);

			mLR_PID.setOutputRange(
					-CamAlignerConstants.PID_Constants.Prototype.LR_MAX, 
					CamAlignerConstants.PID_Constants.Prototype.LR_MAX);

			mLR_PID.setDeadband(CamAlignerConstants.PID_Constants.Prototype.LR_DEADBAND);
			mLR_PID.setIZone(CamAlignerConstants.PID_Constants.Prototype.LR_IZONE);

			LR_TOLERANCE = CamAlignerConstants.PID_Constants.Prototype.LR_TOLERANCE;
			FORWARD_VEL_INITIAL = CamAlignerConstants.PID_Constants.Prototype.FORWARD_VEL_INITIAL;
		}
		else
		{
			mLR_PID = new LocalPIDController(
					CamAlignerConstants.PID_Constants.Real.LR_P,
					CamAlignerConstants.PID_Constants.Real.LR_I,
					CamAlignerConstants.PID_Constants.Real.LR_D,
					mLR_PIDInput, mLR_PIDOutput);

			mLR_PID.setOutputRange(
					-CamAlignerConstants.PID_Constants.Real.LR_MAX, 
					CamAlignerConstants.PID_Constants.Real.LR_MAX);

			mLR_PID.setDeadband(CamAlignerConstants.PID_Constants.Real.LR_DEADBAND);

			LR_TOLERANCE = CamAlignerConstants.PID_Constants.Real.LR_TOLERANCE;
			FORWARD_VEL_INITIAL = CamAlignerConstants.PID_Constants.Real.FORWARD_VEL_INITIAL;
		}

		mLR_PID.setInputRange(-1.0, 1.0);
		mLR_PID.setSetpoint(CamAlignerConstants.LR_SETPOINT);
		mLR_PID.enable();

		mLRInRange = new PersistentlyInRange(mLR_PID, CamAlignerConstants.MIN_TIME_ALIGNED, LR_TOLERANCE);
	}

	public void stop()
	{
		super.stop();
		mLRInRange = null;
		mLR_PID = null;
		mLR_PIDInput = null;
		mLR_PIDOutput = null;
	}

	public DriveTrain.SwerveMovement getMovement() 
	{
		DriveTrain.SwerveMovement finalMovement = new DriveTrain.SwerveMovement();

		boolean validTarget1 = JetsonData.jetsonSeesTarget();
		double lr = JetsonData.jetsonLR();
		double distance = JetsonData.jetsonDistance();
		boolean validTarget2 = JetsonData.jetsonSeesTarget();

		//has to be alid before AND after
		boolean validTarget = validTarget1 && validTarget2;

		handleStates (validTarget, distance);

		switch (mAlignState)
		{
		case NONE:
			finalMovement = DriveTrain.WHEELS_LR;
			break;
		case SEARCHING:
			finalMovement = DriveTrain.WHEELS_LR;
			break;
		case FORWARD_INITIAL:
			finalMovement = forwardInitialMovement(lr);
			break;
		case PREPARE_LR:
			finalMovement = DriveTrain.WHEELS_LR;
			break;
		case ALIGN_LR_PERFECT:
			finalMovement = alignLRPerfectMovement(lr);
			break;
		case COMPLETE:
			finalMovement = DriveTrain.WHEELS_FB;
			break;
		}

		SmartDashboard.putString("Alignment State:", mAlignState.toString());
		SmartDashboard.putBoolean("Valid Target:", validTarget);
		SmartDashboard.putBoolean("Aligned Visoin LR", mLRInRange.inRangePersistently());

		if (mAlignState == AlignState.FORWARD_INITIAL ||
				mAlignState == AlignState.ALIGN_LR_PERFECT)
		{
			SmartDashboard.putNumber("Target LR:", lr);
		}
		SmartDashboard.putNumber("Target Distance:", distance);
		SmartDashboard.putNumber("Output LR", finalMovement.mRobotVel.getY());
		SmartDashboard.putNumber("Output Angle:", finalMovement.mAngularVel);

		return finalMovement;
	}

	private void handleStates(boolean pValidTarget, double distance)
	{
		mLRInRange.inRangePersistently(); //check it to update it

		if (!pValidTarget)
		{
			if (mAlignState == AlignState.FORWARD_INITIAL ||
					mAlignState == AlignState.ALIGN_LR_PERFECT)
			{
				mAlignState = AlignState.SEARCHING;
			}
		}
		switch (mAlignState)
		{
		case NONE:
			mAlignState = AlignState.SEARCHING;
			break;
		case SEARCHING:
			if (pValidTarget)
			{
				mAlignState = AlignState.FORWARD_INITIAL;
				mDriveTrain.resetDriftCompensationPID();
			}
			break;
		case FORWARD_INITIAL:
			if (distance < CamAlignerConstants.DISTANCE_SETPOINT_INITIAL)
			{
				if (mLRInRange.inRangePersistently())
				{
					mAlignState = AlignState.COMPLETE;
				}
				else
				{
					mAlignState = AlignState.PREPARE_LR;
					mLR_PIDInput.setVal(-1000);
					mLRInRange.reset();
					mLR_PID.reset();
					mLR_PID.enable();
					mDriveTrain.resetDriftCompensationPID();
				}
			}
			break;
		case PREPARE_LR:
			if (mDriveTrain.getAllWheelsInRange())
			{
				mAlignState = AlignState.ALIGN_LR_PERFECT;
			}
			break;
		case ALIGN_LR_PERFECT:
			if (mLRInRange.inRangePersistently())
			{
				mAlignState = AlignState.COMPLETE;
			}
			break;
		case COMPLETE:
			break;
		default:
			break;
		}

	}

	private DriveTrain.SwerveMovement forwardInitialMovement (double lr)
	{
		double off = CamAlignerConstants.LR_SETPOINT - lr;
		double output = off * 1.5; //just a P for now
		double angularVel = mDriveTrain.getDriftCompensationAngularVel();

		return new DriveTrain.SwerveMovement(
				angularVel, 
				new Vector (FORWARD_VEL_INITIAL, output), 
				SwerveState.HOLD_FOR_DIRECTION, SpeedMode.SLOW);
	}

	private DriveTrain.SwerveMovement alignLRPerfectMovement (double lr)
	{
		mLR_PIDInput.setVal(lr);
		double output = mLR_PID.get();
		if (output > 0)
		{
			output += CamAlignerConstants.PID_Constants.Prototype.LR_MIN;
		}
		else if (output < 0)
		{
			output -= CamAlignerConstants.PID_Constants.Prototype.LR_MIN;
		}
		double angularVel = mDriveTrain.getDriftCompensationAngularVel();


		return new DriveTrain.SwerveMovement(
				angularVel, 
				new Vector (0, output), 
				SwerveState.HOLD_FOR_DIRECTION, 
				SpeedMode.SLOW);
	}

	public boolean isComplete()
	{
		return mAlignState != AlignState.COMPLETE;
	}

}