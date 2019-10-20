package robotcode.macros;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.Vector;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement;
import robotcode.pid.PersistentlyInRange;

public class RotateToAngleMacro extends DriveMacro {
	private double
	mTargetAngle, 
	mTolerance;
	
	private long mMinTime;
				
	private PersistentlyInRange mGyroInRange;
	
	public RotateToAngleMacro(DriveTrain pDriveTrain, 
			double pTargetAngle, double pTolerance, long pMinTime) {
		super(pDriveTrain);
		mTargetAngle = pTargetAngle;
		mTolerance = pTolerance;
		mMinTime = pMinTime;
	}
	
	public void start()
	{
		super.start();
		mGyroInRange = mDriveTrain.getGyroInRange (mMinTime, mTolerance);
	}
	
	public void stop()
	{
		super.stop();
		mGyroInRange = null;
	}

	@Override
	protected SwerveMovement getMovement() {
		double angularVel = mDriveTrain.getAngularPIDVel(mTargetAngle);
		SmartDashboard.putNumber("Rotate TO Angle Angle:", mDriveTrain.getAngleDegrees());
		SmartDashboard.putNumber("Rotate TO Angle VEL:", angularVel);
		return new SwerveMovement(angularVel, new Vector(), SpeedMode.SLOW);
	}

	@Override
	public boolean isComplete() {
		return mGyroInRange.inRangePersistently();
	}

}
