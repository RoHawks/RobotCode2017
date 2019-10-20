package robotcode.macros;

import resource.Vector;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;

public class SetWheelDirectionMacro extends DriveMacro {

	private double mAngularVel;
	private Vector mRobotVel;
	private boolean mHasRun;
	
	public SetWheelDirectionMacro(
			DriveTrain pDriveTrain,
			double pAngularVel, Vector pRobotVel) {
		super(pDriveTrain);
		
		mAngularVel = pAngularVel;
		mRobotVel = pRobotVel;
		mHasRun = false;
	}

	public void start()
	{
		super.start();
		mHasRun = false;
	}
	
	@Override
	protected SwerveMovement getMovement() {
		mHasRun = true;
		return new SwerveMovement(mAngularVel, mRobotVel, SwerveState.STAY_STILL, SpeedMode.INDIFFERENT);
	}

	@Override
	public boolean isComplete() {
		return mHasRun && mDriveTrain.getAllWheelsInRange();
	}
	
}
