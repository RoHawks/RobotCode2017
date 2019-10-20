package robotcode.macros;

import java.util.function.Consumer;

import org.usfirst.frc.team3419.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MacroRoutine extends Macro {
	private Macro[] mMacroList;
	private int mMacroOn;
	private Consumer <Robot> mStartLambda, mStopLambda;
	private Robot mRobot;

	public MacroRoutine (Macro[] pMacroList)
	{
		this (pMacroList, (Robot r) -> {}, (Robot r) -> {}, null);
	}
	
	public MacroRoutine (Macro[] pMacroList, Consumer <Robot> pStart, Consumer <Robot> pStop, Robot pRobot)
	{
		mMacroList = pMacroList;
		mStartLambda = pStart;
		mStopLambda = pStop;
		mRobot = pRobot;
	}
	
	public void start()
	{
		super.start();
		mMacroOn = 0;
		if (mRobot != null)
		{
			mStartLambda.accept(mRobot);
		}
		mMacroList[0].start();
	}
	
	public void stop()
	{
		super.stop();
		if (mMacroOn < mMacroList.length)
		{
			mMacroList[mMacroOn].stop();
		}
		if (mRobot != null)
		{
			mStopLambda.accept(mRobot);
		}
		
	}

	@Override
	public void runFrame() {
		SmartDashboard.putNumber("Routine Index On:", mMacroOn);
		if (mMacroOn < mMacroList.length)
		{
			if (mMacroList[mMacroOn].isComplete()) 
			{
				mMacroList[mMacroOn].stop();
				mMacroOn++;
				if (mMacroOn < mMacroList.length)
				{
					mMacroList[mMacroOn].start();
				}
			}
			if (mMacroOn < mMacroList.length)
			{
				mMacroList[mMacroOn].runFrame();
			}
		}
	}

	@Override
	public boolean isComplete() {
		return mMacroOn == mMacroList.length;
	}
	
}
