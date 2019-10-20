package robotcode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import robotcode.driving.Wheel;
import sensors.RobotAngle;

public class VelocityEstimator implements Runnable {
	private Wheel[] mWheels;
	private RobotAngle mRobotAngle;
	private JetsonTX1UDP_Client mUDPClient;
	
	public VelocityEstimator (Wheel[] pWheels, RobotAngle pRobotAngle, JetsonTX1UDP_Client pUDPClient)
	{
		mWheels = pWheels;
		mRobotAngle = pRobotAngle;
		mUDPClient = pUDPClient;
	}
	
	public double getAngle ()
	{
		return ResourceFunctions.putAngleInRange(mRobotAngle.getAngleDegrees());
	}
	
	public double getAngularVelocity ()
	{
		return mRobotAngle.getAngularVelocity();
	}

	public double getLRVelocity_Wheels ()
	{
		double yVel = 0;
		for (Wheel w : mWheels)
		{
			yVel += w.getVec().getY() * 12.0; //ft/s to in/s
		}
		
		yVel /= 4.0;
		
		return yVel;
	}
	
	public double getFBVelocity_Wheels ()
	{
		double xVel = 0;
		for (Wheel w : mWheels)
		{
			xVel += w.getVec().getX() * 12.0; //ft/s to in/s
		}
		
		xVel /= 4.0;
		
		return xVel;
	}
	
	public void logToSD ()
	{
		SmartDashboard.putNumber("LR Velocity", getLRVelocity_Wheels());
		SmartDashboard.putNumber("FB Velocity", getFBVelocity_Wheels());
		SmartDashboard.putNumber("Angular Velocity", getAngularVelocity());
	}
	
	private void sendToJetson ()
	{
		String dataString = String.format(
				"SUCCESS,%f,%f,%f,%f,END", 
				getLRVelocity_Wheels(),
				getFBVelocity_Wheels(),
				getAngularVelocity(),
				getAngle());
		
		mUDPClient.sendPacket(dataString);
	}

	@Override
	public void run() {
		for (@SuppressWarnings("unused")
		long frame = 0; ; frame++)
		{
			if (Thread.interrupted())
			{
				break;
			}
			
			//findDisplacement ();
			logToSD();
			sendToJetson();
			Timer.delay(0.01);
		}
	}
	
	
	
}
