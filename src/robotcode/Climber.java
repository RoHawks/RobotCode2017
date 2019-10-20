package robotcode;

import com.ctre.CANTalon;

import constants.ClimberConstants;
import constants.ControlPorts;
import constants.RunConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
	private SpeedController mWinchMotor;
	private Joystick mStick;
	private Encoder mWinchEncoder;
	private DigitalInput mWinchBreakbeam;
	private PowerDistributionPanel mPDP;
	private ClimberState mClimberState;
	
	private double mCurrentEstimate;
	
	private boolean mCurrentTooHigh;
	
	public Climber (
			SpeedController pClimberMotor, 
			Joystick pSecondaryStick, 
			Encoder pEncoder, 
			DigitalInput pBreakbeam)
	{
		mWinchMotor = pClimberMotor;
		mStick = pSecondaryStick;
		mWinchEncoder = pEncoder;
		mWinchBreakbeam = pBreakbeam;
		mCurrentEstimate = -1;
		mClimberState = ClimberState.NONE;
		mCurrentTooHigh = false;
	}
	
	private enum ClimberState
	{
		NONE,
		MANUAL_UP,
		SEARCHING,
		ZONE_1,
		ZONE_2,
		ZONE_3,
		STOPPED
	}
	
	public void setPDP (PowerDistributionPanel pPDP)
	{
		mPDP = pPDP;
	}
	
	private void updateCurrentEstimate()
	{
		double current = 0;
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			current = mPDP.getCurrent(ClimberConstants.PDP_CHANNEL);
		}
		else
		{
			current = ((CANTalon) mWinchMotor).getOutputCurrent();
		}
		
		
		if (mCurrentEstimate < 0)
		{
			mCurrentEstimate = current;
		}
		else if (current > mCurrentEstimate * 1.5)
		{
			mCurrentTooHigh = true;
		}
		else
		{
			mCurrentEstimate = ClimberConstants.GAMMA * current + (1 - ClimberConstants.GAMMA) * mCurrentEstimate;
			mCurrentTooHigh = false;
		}
		
	}
	
	public double getClimberSpeedNoPID()
	{
		double climberSpeed = 0;
		for (int i = 0; i < 10; i++)
		{
			handleStates();
		}
		
		switch (mClimberState)
		{
		case NONE:
			climberSpeed = 0;
			break;
		case MANUAL_UP:
			climberSpeed = ClimberConstants.MANUAL_UP_SPEED;
			break;
		case SEARCHING:
			climberSpeed = ClimberConstants.SPEED_FIND_ROPE;
			break;
		case ZONE_1:
			climberSpeed = ClimberConstants.SPEED_ZONE_1;
			break;
		case ZONE_2:
			climberSpeed = ClimberConstants.SPEED_ZONE_2;
			break;
		case ZONE_3:
			updateCurrentEstimate();
			climberSpeed = ClimberConstants.SPEED_ZONE_3;
			break;
		case STOPPED:
			climberSpeed = 0;
			break;
		}
		
		return climberSpeed;
	}
	
	private void handleStates()
	{
		if (mStick.getRawButton(ControlPorts.MANUAL_CLIMB_PORT))
		{
			mClimberState = ClimberState.MANUAL_UP;
		}
		else if (mStick.getRawButton(ControlPorts.AUTOMATIC_CLIMB_PORT))
		{
			switch (mClimberState)
			{
			case NONE:
				mClimberState = ClimberState.SEARCHING;
				break;
			case MANUAL_UP:
				break;
			case SEARCHING:
//				if (!mWinchBreakbeam.get())
//				{
//					if (mWinchEncoder.getDistance() < ClimberConstants.ZONE_1)
//					{
//						mClimberState = ClimberState.ZONE_1;
//					}
//					else if (mWinchEncoder.getDistance() < ClimberConstants.ZONE_2)
//					{
//						mClimberState = ClimberState.ZONE_2;
//					}
//					else if (mWinchEncoder.getDistance() < ClimberConstants.ZONE_3)
//					{
//						mClimberState = ClimberState.ZONE_3;
//					}
//					else
//					{
//						mClimberState = ClimberState.STOPPED;
//					}
//				}
				break;
			case ZONE_1:
				if (mWinchEncoder.getDistance() > ClimberConstants.ZONE_1)
				{
					mClimberState = ClimberState.ZONE_2;
				}
				break;
			case ZONE_2:
				if (mWinchEncoder.getDistance() > ClimberConstants.ZONE_2)
				{
					mClimberState = ClimberState.ZONE_3;
				}
				break;
			case ZONE_3:
				if (mWinchEncoder.getDistance() > ClimberConstants.ZONE_3 || mCurrentTooHigh)
				{
					mClimberState = ClimberState.STOPPED;
				}
				break;
			case STOPPED:
				break;
					
			}
		}
		else
		{
			mClimberState = ClimberState.NONE;
		}
	}
	
	public void update()
	{
		if (mWinchBreakbeam.get())
		{
			mWinchEncoder.reset();
		}
		
		double climberSpeed = getClimberSpeedNoPID();
		mWinchMotor.set (climberSpeed);
		
		SmartDashboard.putNumber("Climber Winch Height:", mWinchEncoder.getDistance());
		SmartDashboard.putNumber("Climber Winch Speed:", climberSpeed);
		SmartDashboard.putString("Climber Winch State:", mClimberState.toString());
		SmartDashboard.putNumber("Climber Current Estimate:", mCurrentEstimate);
		SmartDashboard.putBoolean("Climber Current Too High:", mCurrentTooHigh);
	}
	
	public void updateSimple ()
	{
		double speed = 0;
		if (mStick.getRawButton(ControlPorts.MANUAL_CLIMB_PORT))
		{
			speed = ClimberConstants.MANUAL_UP_SPEED;
		}
		else if (mStick.getRawButton(ControlPorts.AUTOMATIC_CLIMB_PORT))
		{
			speed = ClimberConstants.SPEED_FIND_ROPE;
		}
		else
		{
			speed = 0.0;
		}
		
		mWinchMotor.set(speed);
	}
}
