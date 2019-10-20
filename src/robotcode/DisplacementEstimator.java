package robotcode;

import com.ctre.CANTalon;

import constants.DriveConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import sensors.RobotAngle;
import sensors.RotationInputter;

public class DisplacementEstimator implements Runnable {
	private static final int AMT_HISTORY_KEEP = 1024;
	
	private Displacement mDisplacement;
	private DisplacementAtTime[] mDisplacementHistory;
	private AngleAtTime[] mAngleHistory;
	
	private int mDisplacementHistoryOn = 0;
	private int mAngleHistoryOn = 0;
	
	private CANTalon[] mDriveTalons;
	private RotationInputter[] mWheelAngles;
	private RobotAngle mRobotAngle;
	
	private double[] prevWheelDistances;

	private boolean mShouldRun = true;
	
	private static final double MAX_ADD = 4.0;
	
	public class Displacement
	{
		public double x, y;
		public Displacement (double pX, double pY)
		{
			x = pX;
			y = pY;
		}
		
		public Displacement copy ()
		{
			return new Displacement (x, y);
		}
	}
	
	private class DisplacementAtTime
	{
		public Displacement displacement;
		public long timestamp;
		
		public DisplacementAtTime (Displacement pDisplacement, long pTimestamp)
		{
			displacement = pDisplacement;
			timestamp = pTimestamp;
		}
		
		public DisplacementAtTime ()
		{
			this (new Displacement (0, 0), -1);
		}
	}
	
	private class AngleAtTime 
	{
		public double angle;
		public long timestamp;
		
		public AngleAtTime (double pAngle, long pTimestamp)
		{
			angle = pAngle;
			timestamp = pTimestamp;
		}
		
		public AngleAtTime ()
		{
			this (0, -1);
		}
	}
	
	public DisplacementEstimator (CANTalon[] pDriveTalons, RotationInputter[] pWheelAngles, RobotAngle pRobotAngle)
	{
		mDisplacement = new Displacement(0, 0);
		
		mDriveTalons = pDriveTalons;
		mWheelAngles = pWheelAngles;
		mRobotAngle = pRobotAngle;
		
		prevWheelDistances = new double[] {0, 0, 0, 0};
		
		
		mDisplacementHistory = new DisplacementAtTime[AMT_HISTORY_KEEP];
		for (int i = 0; i < AMT_HISTORY_KEEP; i++)
		{
			mDisplacementHistory[i] = new DisplacementAtTime();
		}
		
		mAngleHistory = new AngleAtTime[AMT_HISTORY_KEEP];
		for (int i = 0; i < AMT_HISTORY_KEEP; i++)
		{
			mAngleHistory[i] = new AngleAtTime();
		}
	}
	
	public DisplacementEstimator (RotationInputter[] pWheelAngles, RobotAngle pRobotAngle)
	{
		this (null, pWheelAngles, pRobotAngle);
	}
	
	public synchronized double getHistoricalAngle (long time)
	{
		double closestAngle = 0;
		long bestDif = Long.MAX_VALUE;

		for (int i = 0; i < AMT_HISTORY_KEEP; i++)
		{
			if (mAngleHistory[i].timestamp > 0)
			{
				long dif = Math.abs (mAngleHistory[i].timestamp - time);

				if (dif < bestDif)
				{
					closestAngle = mAngleHistory[i].angle;
					bestDif = dif;
				}
			}
		}

		return closestAngle;
	}
	
	public synchronized Displacement getDisplacementDif (long time1, long time2)
	{
		Displacement closestT1 = new Displacement(0, 0);
		Displacement closestT2 = new Displacement(0, 0);
		long bestDif1 = Long.MAX_VALUE;
		long bestDif2 = Long.MAX_VALUE;

		long bestTimestamp1 = -1;
		long bestTimestamp2 = -1;


		for (int i = 0; i < AMT_HISTORY_KEEP; i++)
		{
			if (mDisplacementHistory[i].timestamp > 0)
			{
				long dif1 = Math.abs (mDisplacementHistory[i].timestamp - time1);
				long dif2 = Math.abs (mDisplacementHistory[i].timestamp - time2);

				if (dif1 < bestDif1)
				{
					closestT1 = mDisplacementHistory[i].displacement;
					bestDif1 = dif1;
					bestTimestamp1 = mDisplacementHistory[i].timestamp;
				}

				if (dif2 < bestDif2)
				{
					closestT2 = mDisplacementHistory[i].displacement;
					bestDif2 = dif2;
					bestTimestamp2 = mDisplacementHistory[i].timestamp;
				}
			}
		}


		SmartDashboard.putNumber("Best timestamp 1:", bestTimestamp1);
		SmartDashboard.putNumber("Best timestamp 2:", bestTimestamp2);

		SmartDashboard.putString("Best Timestamp difference 1, 2; val 1, 2", 
				String.format("%d, %d; %.2f, %.2f", bestDif1, bestDif2, closestT1.x, closestT2.x));

		if (bestDif1 == Long.MAX_VALUE || bestDif2 == Long.MAX_VALUE)
		{
			return new Displacement (0, 0);
		}
		else
		{
			return new Displacement (closestT2.x - closestT1.x, closestT2.y - closestT1.y);
		}
	}
	
	private void recordDisplacement (Displacement pDisplacement, long curTime)
	{
		mDisplacementHistory[mDisplacementHistoryOn] = new DisplacementAtTime(pDisplacement.copy(), curTime);
		mDisplacementHistoryOn++;
		mDisplacementHistoryOn %= AMT_HISTORY_KEEP;
	}
	
	private void recordAngle (double pAngle, long curTime)
	{
		mAngleHistory[mAngleHistoryOn] = new AngleAtTime(pAngle, curTime);
		mAngleHistoryOn++;
		mAngleHistoryOn %= AMT_HISTORY_KEEP;
	}
	
	private void updateDisplacement ()
	{
		double x = 0;
		double y = 0;
		
		for (int i = 0; i < 4; i++)
		{
			double amtRotations = (mDriveTalons == null) ?
					0.0 :  mDriveTalons[i].getPosition() - prevWheelDistances[i];
			
			x += amtRotations * Math.cos (Math.toRadians (mWheelAngles[i].getAngleDegreesNoAdd180()));
			y += amtRotations * Math.sin (Math.toRadians (mWheelAngles[i].getAngleDegreesNoAdd180()));
			
			if (mDriveTalons != null)
			{
				prevWheelDistances[i] = mDriveTalons[i].getPosition();
			}
		}

		boolean distanceValid = x < MAX_ADD && y < MAX_ADD;

		//account for number of wheels
		x /= 4.0;
		y /= 4.0;

		//gearbox & feet to inches
		x *= DriveConstants.RAW_TO_RPS * 12; 
		y *= DriveConstants.RAW_TO_RPS * 12;

		if (!distanceValid)
		{
			return;
		}
		else
		{
			mDisplacement.x += x; //x;
			mDisplacement.y += y;
			SmartDashboard.putNumber("Displacement X:", mDisplacement.x);
			SmartDashboard.putNumber("Displacement Y:", mDisplacement.x);
			recordDisplacement (mDisplacement, System.currentTimeMillis());
		}
		
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
		if (RunConstants.DONT_RUN_THREADS) mShouldRun = false;
		if (!RunConstants.RUNNING_DISPLACEMENT_ESTIMATOR) mShouldRun = false;
		for (@SuppressWarnings("unused")long frame = 0; ; frame++)
		{
			if (Thread.interrupted() || !mShouldRun)
			{
				break;
			}
			
			updateDisplacement();
			recordAngle(mRobotAngle.getAngleDegrees(), System.currentTimeMillis());
			Timer.delay(0.005);
		}
	}
}
