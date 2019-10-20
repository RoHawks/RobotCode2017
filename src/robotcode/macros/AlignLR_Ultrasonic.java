package robotcode.macros;

import constants.RunConstants;
import constants.UltrasonicAlignerConstants;
import edu.wpi.first.wpilibj.LocalPIDController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.Vector;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;
import robotcode.pid.GenericPIDOutput;
import robotcode.pid.PersistentlyInRange;

public class AlignLR_Ultrasonic extends DriveMacro {
	private LocalPIDController mLR_PID;
	private GenericPIDOutput mLR_PIDOutput;
	private PersistentlyInRange mLRInRange;
	
	private Ultrasonic mUltrasonic;
	private AlignState mAlignState;
	
	public static int mNumIterated;
	private double mTolerance;
	
	public enum AlignState {
		NONE,
		SEARCHING,
		ALIGN_LR,
		COMPLETE
	}
	public AlignLR_Ultrasonic(
			DriveTrain pDriveTrain, 
			Ultrasonic pUltrasonic,
			double pTolerance) {
		
		super(pDriveTrain);
		mUltrasonic = pUltrasonic;
		mTolerance = pTolerance;
	}
	
	public void start()
	{
		super.start();
		mAlignState = AlignState.NONE;
		mLR_PIDOutput = new GenericPIDOutput();
		
		
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			mLR_PID = new LocalPIDController(
					UltrasonicAlignerConstants.Prototype.PID_P, 
					UltrasonicAlignerConstants.Prototype.PID_I, 
					UltrasonicAlignerConstants.Prototype.PID_D, 
					mUltrasonic, mLR_PIDOutput);
			mLR_PID.setSetpoint(UltrasonicAlignerConstants.PID_SETPOINT);
			mLR_PID.setOutputRange(
					-UltrasonicAlignerConstants.Prototype.PID_MAX, 
					UltrasonicAlignerConstants.Prototype.PID_MAX);
			mLR_PID.setIZone(UltrasonicAlignerConstants.Prototype.PID_IZONE);
		}
		else
		{
			mLR_PID = new LocalPIDController(
					UltrasonicAlignerConstants.Real.PID_P, 
					UltrasonicAlignerConstants.Real.PID_I, 
					UltrasonicAlignerConstants.Real.PID_D, 
					mUltrasonic, mLR_PIDOutput);
			mLR_PID.setSetpoint(UltrasonicAlignerConstants.PID_SETPOINT);
			mLR_PID.setOutputRange(
					-UltrasonicAlignerConstants.Real.PID_MAX, 
					UltrasonicAlignerConstants.Real.PID_MAX);
			mLR_PID.setIZone(UltrasonicAlignerConstants.Real.PID_IZONE);
		}
		
		mLR_PID.enable();
		
		mLRInRange = new PersistentlyInRange(
				mLR_PID, 
				(long) UltrasonicAlignerConstants.MIN_TIME_IN_RANGE, 
				mTolerance);
		
		mNumIterated = 0;
	}
	
	public void stop()
	{
		super.stop();
		mLRInRange = null;
		mLR_PID = null;
		mLR_PIDOutput = null;
	}

	public DriveTrain.SwerveMovement getMovement() 
	{
		handleStates();
		
		SmartDashboard.putNumber("Num Ultrasonic Iterated:", mNumIterated);
		mNumIterated++;
		
		DriveTrain.SwerveMovement finalMovement = new DriveTrain.SwerveMovement();
		switch (mAlignState)
		{
		case NONE:
			finalMovement = DriveTrain.WHEELS_LR;
			break;
		case SEARCHING:
			finalMovement = DriveTrain.WHEELS_LR;
			break;
		case ALIGN_LR:
			finalMovement = alignLRMovement();
			break;
		case COMPLETE:
			finalMovement = DriveTrain.WHEELS_LR;
			break;
		}
		
		SmartDashboard.putString("Alignment State Ultrasonic:", mAlignState.toString());
		SmartDashboard.putNumber("Ultrasonic Alignment LR", finalMovement.mRobotVel.getY());
		SmartDashboard.putNumber("Peg LR:", mUltrasonic.getRangeInches());
		SmartDashboard.putBoolean("Peg In Range:", mLRInRange.inRange());
		SmartDashboard.putNumber("Ultrasonic LR PID Error:", mLR_PID.getError());
		return finalMovement;
	}
	
	private DriveTrain.SwerveMovement alignLRMovement()
	{
		double output = mLR_PID.get();
		if (mUltrasonic.getRangeInches() > UltrasonicAlignerConstants.MAX_RANGE_INCHES)
		{
			return DriveTrain.WHEELS_LR;
		}
		else
		{
			return new DriveTrain.SwerveMovement(0, new Vector (0, output), SwerveState.NORMAL, SpeedMode.SLOW);
		}	
	}
	
	private void handleStates()
	{
		switch (mAlignState)
		{
		case NONE:
			mAlignState = AlignState.SEARCHING;
			break;
		case SEARCHING:
			if (mUltrasonic.getRangeInches() < UltrasonicAlignerConstants.MAX_RANGE_INCHES)
			{
				mAlignState = AlignState.ALIGN_LR;
			}
			break;
		case ALIGN_LR:
			if (mLRInRange.inRangePersistently())
			{
				mAlignState = AlignState.COMPLETE;
			}
			if (mUltrasonic.getRangeInches() > UltrasonicAlignerConstants.MAX_RANGE_INCHES)
			{
				mAlignState = AlignState.SEARCHING;
			}
			break;
		case COMPLETE:
			break;
		}
	}
	
	public boolean isComplete()
	{
		return mAlignState == AlignState.COMPLETE;
	}
}
