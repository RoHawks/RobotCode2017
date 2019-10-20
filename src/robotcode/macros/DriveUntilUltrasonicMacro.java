package robotcode.macros;

import constants.UltrasonicAlignerConstants;
import edu.wpi.first.wpilibj.Ultrasonic;
import resource.Vector;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement;

public class DriveUntilUltrasonicMacro extends DriveMacro {
	private Ultrasonic mUltrasonic;
	private long mRampUp, mTimeStarted;
	private double mSpeed;
	private long mMaxTime;
	private long mTimeSeenPeg;
	
	public DriveUntilUltrasonicMacro(DriveTrain pDriveTrain, Ultrasonic pUltrasonic, long pRampUp, double pSpeed, long pMaxTime) {
		super (pDriveTrain);
		mUltrasonic = pUltrasonic;

		mRampUp = pRampUp;
		mSpeed = pSpeed;
		mMaxTime = pMaxTime;
	}
	
	public void start()
	{
		mTimeStarted = System.currentTimeMillis();
		mTimeSeenPeg = -1;
	}
	
	@Override
	protected SwerveMovement getMovement() {
		double speed = mSpeed;
		long dif = System.currentTimeMillis() - mTimeStarted;
		
		if (dif < mRampUp)
		{
			double mult = (double)(dif) / (double)(mRampUp);
			speed *= mult;
		}
		
		if (mTimeSeenPeg < 0 && mUltrasonic.getRangeInches() < UltrasonicAlignerConstants.MAX_RANGE_INCHES)
		{
			mTimeSeenPeg = System.currentTimeMillis();
		}
		
		return new SwerveMovement(0, Vector.createPolar(0, speed), SpeedMode.SLOW);
	}

	@Override
	public boolean isComplete() {
		long dif = System.currentTimeMillis() - mTimeStarted;
		boolean complete = false;
		if (dif > mMaxTime)
		{
			complete = true;
		}
		
		if (System.currentTimeMillis() - mTimeSeenPeg > 100 && mTimeSeenPeg > 0)
		{
			complete = true;
		}
		
		return complete;
		
	}
	

}