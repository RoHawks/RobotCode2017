package robotcode.macros;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;

public class LinearMotionMacro extends DriveMacro
{
	private Vector mVelocity;
	private long mRampTime;
	private boolean mHoldAngle;
	private boolean mNudge, mIsMoving, mFieldRelative;
	private SpeedMode mSpeedMode;
	private long mDuration, mTimeStarted;

	public LinearMotionMacro(
			DriveTrain pDriveTrain, 
			long pDuration, Vector pVelocity, 
			long pRampTime, boolean pHoldAngle,
			boolean pNudge, boolean pFieldRelative, 
			SpeedMode pSpeedMode) {

		super(pDriveTrain);
		mVelocity = pVelocity;
		mRampTime = pRampTime;
		mHoldAngle = pHoldAngle;
		mNudge = pNudge;
		mFieldRelative = pFieldRelative;
		mSpeedMode = pSpeedMode;
		mDuration = pDuration;
	}

	public Vector getVelocity()
	{
		return mVelocity;
	}

	public void start()
	{
		super.start();
		mIsMoving = false;
		mTimeStarted = -1;
		if (mHoldAngle)
		{
			mDriveTrain.resetDriftCompensationPID();
		}
	}

	private long currentAmountExecuted()
	{
		return System.currentTimeMillis() - mTimeStarted;
	}
	
	@Override
	public SwerveMovement getMovement() {
		double speed = 0;
		double angle = mVelocity.getAngle();
		
		
		if (mFieldRelative)
		{
			double robotAngle = mDriveTrain.getAngleDegrees();
			angle -= robotAngle; //make it field relative
			angle = ResourceFunctions.putAngleInRange(angle);
		}
		SmartDashboard.putBoolean("Moving Linear Motion:", mIsMoving);
		if (!mIsMoving && ((mNudge && mDriveTrain.getAllWheelsInRange()) || !mNudge))
		{
			mIsMoving = true;
			mTimeStarted = System.currentTimeMillis();
		}
		
		if (mIsMoving)
		{
			long amountExecuted = currentAmountExecuted();
			if (mRampTime > 0)
			{
				double mult1 = (double)amountExecuted / (double)mRampTime;
				double mult2 = (double)(mDuration - amountExecuted) / (double)mRampTime;
				double mult = Math.min(Math.min(mult1, mult2), 1.0);
				speed = mVelocity.getMagnitude() * mult;
			}
			else
			{
				speed = mVelocity.getMagnitude();
			}
		}
		else
		{
			speed = 1.0; //ok because we aren't driving
		}

		double angularVel = mHoldAngle ? mDriveTrain.getDriftCompensationAngularVel() : 0;
		Vector moveVec = Vector.createPolar(angle, speed);
		
		SwerveMovement swerveMovement = new SwerveMovement();
		if (mIsMoving)
		{
			swerveMovement = new SwerveMovement(angularVel, moveVec, SwerveState.HOLD_FOR_DIRECTION, mSpeedMode);
		}
		else
		{
			swerveMovement = new SwerveMovement(0, moveVec, SwerveState.STAY_STILL, mSpeedMode);
		}

		return swerveMovement;
	}
	
	public boolean isComplete()
	{
		return mIsMoving && (System.currentTimeMillis() - mTimeStarted > mDuration);
	}
}