package robotcode.macros;

import java.util.function.Consumer;
import java.util.function.Predicate;

import org.usfirst.frc.team3419.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public abstract class Macro {
	private boolean mIsRunning = false;
	
	public void start() {
		mIsRunning = true;
	}
	public void stop() {
		mIsRunning = false;
	}
	
	/**
	 * Executes time slice of this macro, must occupy only a small period of time
	 */
	
	public abstract void runFrame();
	public abstract boolean isComplete();
	
	public final boolean execute (Predicate<Robot> pIsDisabled, Robot pRobot)
	{
		return this.execute(pIsDisabled, (Robot r) -> {}, pRobot);
	}
	
	public final boolean execute (Predicate<Robot> pIsDisabled, Consumer <Robot> pOnEachRun, Robot pRobot)
	{
		start();
		while (!isComplete())
		{
			runFrame();
			pOnEachRun.accept(pRobot);
			
			Timer.delay(0.005);
			if (Thread.interrupted() || pIsDisabled.test(pRobot) || pRobot.isDisabled())
			{
				stop();
				return false;
			}
		}
		stop();
		return true;
	}
	
	public final boolean isRunning()
	{
		return mIsRunning;
	}
	
}
