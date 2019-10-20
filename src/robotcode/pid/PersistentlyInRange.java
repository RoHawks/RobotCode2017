package robotcode.pid;

import edu.wpi.first.wpilibj.LocalPIDController;

public class PersistentlyInRange {
	private LocalPIDController mPID;
	private long mMinTimeInRangeMillis;
	private double mTolerance;
	
	private long mFirstTimeInRange;

	public PersistentlyInRange (LocalPIDController pPID, long pMinTimeInRangeMillis, double pTolerance)
	{
		mPID = pPID;
		mMinTimeInRangeMillis = pMinTimeInRangeMillis;
		mTolerance = pTolerance;
		
		mFirstTimeInRange = -1;
	}
	
	public boolean inRange()
	{
		return Math.abs(mPID.getError()) < mTolerance;
	}
	
	public void reset()
	{
		mFirstTimeInRange = -1;
	}
	
	public boolean inRangePersistently()
	{
		boolean inRangePersistently = false;
		if (mMinTimeInRangeMillis < 0)
		{
			inRangePersistently = inRange();
		}
		else
		{
			if (inRange())
			{
				if (mFirstTimeInRange == -1)
				{
					mFirstTimeInRange = System.currentTimeMillis();
				}
				else
				{
					inRangePersistently = 
							System.currentTimeMillis() - mFirstTimeInRange > mMinTimeInRangeMillis;
				}
			}
			else
			{
				mFirstTimeInRange = -1;
			}
		}
		
		return inRangePersistently;
	}
	
	public long timeInRange()
	{
		long timeInRange = -1;
		if (inRange())
		{
			if (mFirstTimeInRange == -1)
			{
				mFirstTimeInRange = System.currentTimeMillis();
			}
			else
			{
				timeInRange = System.currentTimeMillis() - mFirstTimeInRange;
			}
		}
		else
		{
			mFirstTimeInRange = -1;
		}
		
		
		return timeInRange;
	}
	
}
