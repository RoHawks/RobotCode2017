package robotcode.macros;

import edu.wpi.first.wpilibj.Timer;

public class DelayMacro extends Macro {
	private long mDelayTime, mTimeStarted;

	public DelayMacro (long pDelayTime)
	{
		mDelayTime = pDelayTime;
		mTimeStarted = -1;
	}
	
	public void start()
	{
		super.start();
		mTimeStarted = System.currentTimeMillis();
	}
	
	@Override
	public void runFrame() {
		Timer.delay(0.005);
	}

	@Override
	public boolean isComplete() {
		return System.currentTimeMillis() - mTimeStarted > mDelayTime;
	}
	
}
