package robotcode.macros;

import robotcode.driving.DriveTrain;

public abstract class DriveMacro extends Macro
{
	protected DriveTrain mDriveTrain;
	protected abstract DriveTrain.SwerveMovement getMovement();
	
	public DriveMacro (DriveTrain pDriveTrain)
	{
		mDriveTrain = pDriveTrain;
	}
	
	public void runFrame()
	{
		if (this.isComplete())
		{
			mDriveTrain.enactMovement(DriveTrain.DO_NOTHING);
		}
		else
		{
			mDriveTrain.getAllWheelsInRange();
			mDriveTrain.enactMovement(getMovement());
		}
		
	}
	
	public void stop()
	{
		super.stop();
		mDriveTrain.enactMovement(DriveTrain.DO_NOTHING);
	}
}