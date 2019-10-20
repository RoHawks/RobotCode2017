package robotcode;


import constants.RunConstants;
import constants.ZedAlignmentConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import robotcode.DisplacementEstimator.Displacement;
import sensors.RobotAngle;

public class PoseEstimator implements Runnable {
	private JetsonTX1UDP_Server mServer;
	private DisplacementEstimator mDisplacementEstimator;
	
	private double mLastFB_Image, mLastLR_Image;
	private double mFB, mLR;
	
	private long mLastImageTimestamp;
	private boolean mValid;
	
	private double mPegGyroAngleEstimate;
	private double mNumAverages;
	private RobotAngle mRobotAngle;
	private boolean mShouldRun = true;
	
	private static final boolean RESET_COUNT_AT_ERROR = false;
	
	public PoseEstimator (int pLocalPort, int pBytes, RobotAngle pRobotAngle, DisplacementEstimator pDisplacementEstimator)
	{
		mLastImageTimestamp = -1;
		mServer = new JetsonTX1UDP_Server(pLocalPort, pBytes);
		mRobotAngle = pRobotAngle;
		mDisplacementEstimator = pDisplacementEstimator;
		
		mLastFB_Image = 0;
		mLastLR_Image = 0;
		mValid = false;
		
		mPegGyroAngleEstimate = 0;
		mNumAverages = 0;
		
		StringBuilder headerString = new StringBuilder();
		for (int i = 0; i < ZedAlignmentConstants.RobotPositionData.length; i++)
		{
			if (i > 0)
			{
				headerString.append(",");
			}
			headerString.append (ZedAlignmentConstants.RobotPositionData[i]);
		}
		
		SmartDashboard.putString("RobotPositionHeader", headerString.toString());
	}
	
	public synchronized void resetCount ()
	{
		mNumAverages = 0;
	}

	private void updateAngleEstimate ()
	{
		if (!mServer.getValid()) return;
		long timestamp = mServer.getTimestamp();
		double pegAngle = mServer.getAngle();
		
		
		SmartDashboard.putString ("BEEP BOOP BEEP BOOP TRYING TO UPDATE ANGLE", "beeep");
		SmartDashboard.putBoolean ("Displacement Estimate is null", mDisplacementEstimator == null);
		double gyroAtTime = mDisplacementEstimator.getHistoricalAngle(timestamp);
		double prediction = ResourceFunctions.putAngleInRange(gyroAtTime - pegAngle);
		if (prediction > 180)
		{
			prediction -= 360;
		}
		
		double dif = ResourceFunctions.continuousAngleDif(prediction, mPegGyroAngleEstimate);
		if (Math.abs(dif) > ZedAlignmentConstants.MIN_DIF_ERASE_ESTIMATE && RESET_COUNT_AT_ERROR)
		{
			mNumAverages = 0;
		}
		SmartDashboard.putNumber ("Gyro Angle Estimate:", mPegGyroAngleEstimate);
		SmartDashboard.putNumber ("Gyro Angle Estimate Guess:", prediction);
		SmartDashboard.putNumber ("Peg Angle:", pegAngle);
		SmartDashboard.putNumber ("Gyro Angle:", gyroAtTime);
		
		double weightNew = 0.0, weightOld = 1.0;
		synchronized (this)
		{
			mNumAverages++;
			weightNew = 1.0 / mNumAverages;
			weightOld = 1.0 - weightNew;
		}
		
		
		mPegGyroAngleEstimate = ResourceFunctions.putAngleInRange(mPegGyroAngleEstimate);
		if (mPegGyroAngleEstimate > 180)
		{
			mPegGyroAngleEstimate -= 360;
		}
		mPegGyroAngleEstimate = weightNew * prediction + weightOld * mPegGyroAngleEstimate;
		mPegGyroAngleEstimate = ResourceFunctions.putAngleInRange(mPegGyroAngleEstimate);
		if (mPegGyroAngleEstimate > 180)
		{
			mPegGyroAngleEstimate -= 360;
		}
		
	}
	
	public synchronized boolean getValid ()
	{
		return mValid;
	}
	
	public synchronized double getFB ()
	{
		return mFB;
	}
	
	public synchronized double getLR()
	{
		return mLR;
	}
	
	public synchronized double getAngle()
	{
		return ResourceFunctions.putAngleInRange(mRobotAngle.getAngleDegrees() - mPegGyroAngleEstimate);
	}
	
	public synchronized void disable ()
	{
		mShouldRun = false;
	}
	
	public synchronized void enable ()
	{
		mShouldRun = true;
	}
	
	@Override
	public void run() {
		//int prev = 0;
		//performSpeedTest();
		long lastTimeValid = -1;
		
		Thread serverUpdaterThread = new Thread (new Runnable() {
			@Override
			public void run() {
				while (mShouldRun && !Thread.interrupted())
				{
					mServer.updateData();
				}
			}
		});
		
		if (!RunConstants.DONT_RUN_THREADS) serverUpdaterThread.start();
		if (RunConstants.DONT_RUN_THREADS) mShouldRun = false;
		for (long frame = 0; ; frame++)
		{
			if (Thread.interrupted() || !mShouldRun)
			{
				break;
			}
			
			SmartDashboard.putNumber ("Pose Estimator frame", frame);
			//mServer.updateData();
			
			if (mServer.getValid() && mServer.getTimestamp() != mLastImageTimestamp)
			{
				mValid = true;
				updateAngleEstimate();
				mLastFB_Image = mServer.getFB();
				mLastLR_Image = mServer.getLR();
				mLastImageTimestamp = mServer.getTimestamp();
				lastTimeValid = System.currentTimeMillis();
			}
			
			if (System.currentTimeMillis() - lastTimeValid > ZedAlignmentConstants.MILLIS_TO_FORGET_POSE)
			{
				mNumAverages = 0;
				mValid = false;
				mLR = 0;
				mFB = 0;
			}
			else
			{
				SmartDashboard.putNumber ("Timestamp DIfference:", System.currentTimeMillis() - mLastImageTimestamp);
				
				if (RunConstants.USING_DRIVE_ENCODERS_FOR_PREDICTION)
				{
					Displacement dif = mDisplacementEstimator.getDisplacementDif(mLastImageTimestamp, System.currentTimeMillis());
					SmartDashboard.putNumber ("Displacement Dif X:", dif.x);
					SmartDashboard.putNumber ("Displacement Dif Y:", dif.y);
					
					mFB = mLastFB_Image - dif.x;
					mLR = mLastLR_Image + dif.y;
				}
				else
				{
					mFB = mLastFB_Image;
					mLR = mLastLR_Image;
				}
			}
			double angle = getAngle();
			double costheta = Math.cos(Math.toRadians(angle));
			double sintheta = Math.sin(Math.toRadians(angle));
			double pegRelativeLR = -mLR * costheta + mFB * sintheta;
			double pegRelativeFB = mLR * sintheta + mFB * costheta;
			String loggerString = 
					String.format ("%f,%f,%f,%d,%d,%d",
							pegRelativeLR, pegRelativeFB, angle,
							//mLR, mFB, angle,
							0, 0, 0);

			SmartDashboard.putString ("RobotPositionData", loggerString);


			Timer.delay(0.02);
		}
	}
	

}
