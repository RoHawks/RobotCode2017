package robotcode.driving;

import resource.ResourceFunctions;
import resource.Vector;
import robotcode.pid.PersistentlyInRange;
import sensors.RotationInputter;
import sensors.RelativeEncoder;

import com.ctre.CANTalon;

import constants.DriveConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.LocalPIDController;
import edu.wpi.first.wpilibj.SpeedController;

public class Wheel {
	private SpeedController mTurn;
	private SpeedController mDrive;
	
	private double m_xOff;
	private double m_yOff;

	private LocalPIDController mRotationPID;
	private PersistentlyInRange mAngleInRange;
	
	private RotationInputter mRotationSource;
	private RelativeEncoder mSpeedSource;
	
	private boolean mReverseSpeed;

	private boolean mFastMode;
	
	private double 
		mFastSpeedPID_P, mFastSpeedPID_I, mFastSpeedPID_D, mFastSpeedPID_F,
		mSlowSpeedPID_P, mSlowSpeedPID_I, mSlowSpeedPID_D, mSlowSpeedPID_F;
	
	public Wheel(SpeedController mTurnMotors, SpeedController mDriveMotors, 
			double pROT_PID_P, double pROT_PID_I, double pROT_PID_D, double pROT_PID_IZONE,
			double pFAST_SPEED_PID_P, double pFAST_SPEED_PID_I, double pFAST_SPEED_PID_D,  double pFAST_SPEED_PID_F,
			double pSLOW_SPEED_PID_P, double pSLOW_SPEED_PID_I, double pSLOW_SPEED_PID_D,  double pSLOW_SPEED_PID_F,
			double pXOff, double pYOff,
			RotationInputter pTurnEncoder, RelativeEncoder pDriveEncoder) {
		
		//initialize motors
		mTurn = mTurnMotors;
		mDrive = mDriveMotors;
		
		//initialize wheel position from robot center
		m_xOff = pXOff;
		m_yOff = pYOff;
		
		//initialize encoder
		mRotationSource = pTurnEncoder;
		mSpeedSource = pDriveEncoder;
		
		//initialize turn motor PID
		mRotationPID = new LocalPIDController(pROT_PID_P, pROT_PID_I, pROT_PID_D, mRotationSource, mTurn);
		mRotationPID.setContinuous(true); //wrap around 360
		mRotationPID.setInputRange(0, 360);
		mRotationPID.setOutputRange(-1, 1); //may want to set nominal output for minimums
		mRotationPID.setIZone(pROT_PID_IZONE);
		mRotationPID.enable();
		mAngleInRange = new PersistentlyInRange(mRotationPID, DriveConstants.MIN_TIME_IN_RANGE_NUDGE_MILLIS, DriveConstants.NUDGE_ANGLE_TOLERANCE);

		
		mFastSpeedPID_P = pFAST_SPEED_PID_P;
		mFastSpeedPID_I = pFAST_SPEED_PID_I;
		mFastSpeedPID_D = pFAST_SPEED_PID_D;
		mFastSpeedPID_F = pFAST_SPEED_PID_F;
		//mFastSpeedPID_IZone = pFAST_SPEED_PID_IZONE;
		
		mSlowSpeedPID_P = pSLOW_SPEED_PID_P;
		mSlowSpeedPID_I = pSLOW_SPEED_PID_I;
		mSlowSpeedPID_D = pSLOW_SPEED_PID_D;
		mSlowSpeedPID_F = pSLOW_SPEED_PID_F;
		
		mFastMode = false;
		
		if (RunConstants.SPEED_PID)
		{
			setPIDConstants();
		}
		//mSlowSpeedPID_IZone = pSLOW_SPEED_PID_IZONE;
	}
	
	public Wheel(SpeedController mTurnMotors, SpeedController mDriveMotors, 
			double pROT_PID_P, double pROT_PID_I, double pROT_PID_D, double pROT_PID_IZONE,
			double pXOff, double pYOff,
			RotationInputter pTurnEncoder) {
		
		//initialize motors
		mTurn = mTurnMotors;
		mDrive = mDriveMotors;
		
		//initialize wheel position from robot center
		m_xOff = pXOff;
		m_yOff = pYOff;
		
		//initialize encoder
		mRotationSource = pTurnEncoder;
		
		//initialize turn motor PID
		mRotationPID = new LocalPIDController(pROT_PID_P, pROT_PID_I, pROT_PID_D, mRotationSource, mTurn);
		mRotationPID.setContinuous(true); //wrap around 360
		mRotationPID.setInputRange(0, 360);
		mRotationPID.setOutputRange(-1, 1); //may want to set nominal output for minimums
		mRotationPID.setIZone(pROT_PID_IZONE);
		mRotationPID.enable();
		mAngleInRange = new PersistentlyInRange(mRotationPID, DriveConstants.MIN_TIME_IN_RANGE_NUDGE_MILLIS, DriveConstants.NUDGE_ANGLE_TOLERANCE);
	}
	
	private void setPIDConstants() {
		if (RunConstants.SPEED_PID) {
			if (mFastMode) {
				((CANTalon) mDrive).setP(mFastSpeedPID_P);
				((CANTalon) mDrive).setI(mFastSpeedPID_I);
				((CANTalon) mDrive).setD(mFastSpeedPID_D);
				((CANTalon) mDrive).setF(mFastSpeedPID_F);
				//mDrive.setIZone(mFastSpeedPID_IZone / DriveConstants.RAW_TO_RPS);
			}
			
			else {
				((CANTalon) mDrive).setP(mSlowSpeedPID_P);
				((CANTalon) mDrive).setI(mSlowSpeedPID_I);
				((CANTalon) mDrive).setD(mSlowSpeedPID_D);
				((CANTalon) mDrive).setF(mSlowSpeedPID_F);
				//mDrive.setIZone(mSlowSpeedPID_IZone);
			}
		}
	}
	
	public void setSpeedMode (boolean pFastMode) {
		boolean eq = mFastMode == pFastMode;
		mFastMode = pFastMode;
		if (!eq)
		{
			setPIDConstants();
		}
	}
	
	/**
	 * Calculates if we are currently within nudging tolerance
	 * @return true if we are within nudge tolerance, otherwise false
	 */
	public boolean isInRangeNudge() {
		return mAngleInRange.inRangePersistently();
	}
	
	public double getSetpoint() {
		return mRotationPID.getSetpoint();
	}
	
	public double getErr() {
		return mRotationPID.getError();
	}
	
	public double getPIDOutput() {
		return mRotationPID.get();
	}
	
	public double getRealAngle() {
		return mRotationSource.pidGet();
	}

	public Vector getVec() {
		return Vector.createPolar(getRealAngle(), Math.abs(getSpeed()));
	}
	
	public double getSpeed() {
		if (RunConstants.SPEED_PID)
		{
			return mSpeedSource.getRPS();
		
		}
		else
		{
			return mDrive.get() * 2; //for prototype bot; improve this
		}
	}
	
	
	/**
	 * Gets x-distance from the center of the robot
	 * x-axis points NORTH relative to the robot
	 * @return x-distance in inches
	 */
	public double getXOff() {
		return m_xOff;
	}
	
	/**
	 * Gets the y-distance from the center of the robot
	 * y-axis points WEST relative to the robot
	 * @return y-distance in inches
	 */
	public double getYOff() {
		return m_yOff;
	}
	
	/**
	 * Set wheel angle & speed
	 * @param pWheelVelocity Vector of wheel velocity
	 */
	public void set (Vector pWheelVelocity) {
		set (pWheelVelocity.getAngle(), pWheelVelocity.getMagnitude());
	}
	
	/**
	 * Set wheel angle & speed
	 * @param angle direction to point the wheel
	 * @param speed magnitude to drive the wheel
	 */
	public void set(double angle, double speed) {
		setAngle(angle);
		setSpeed(speed);
	}
	
	/**
	 * Sets the wheel angle optimizing for travel time
	 * tells a PID what setpoint to use
	 * @param angle direction to point the wheel
	 */
	public void setAngle(double angle) {
		//find distance from setpoint (as the PID would calculate it)
		/*double dif = mRotationPID.getSetpoint() - angle;
		dif = ResourceFunctions.putAngleInRange(dif);
		if(dif > 180) dif = 360 - dif;*/
		
		double dif = ResourceFunctions.continuousAngleDif(mRotationPID.getSetpoint(), angle);;

		//reverse the wheel direction & add 180 to encoder angle if its faster than turning around
		if(Math.abs(dif) > 92) {
			mReverseSpeed = !mReverseSpeed; 
			mRotationSource.setAdd180(mReverseSpeed);
		}
		//reverse the wheel direction & add 180 to encoder angle if its faster than turning around
		/*if(Math.abs(dif) < 100) 
		{
			mReverseSpeed = !mReverseSpeed; 
			mRotationSource.setAdd180(mReverseSpeed);
		}*/

		mRotationPID.setSetpoint(ResourceFunctions.putAngleInRange(angle));
	}
	
	/**
	 * Sets the speed of the wheel including multipliers
	 * @param speed raw speed of the wheel
	 */
	public void setSpeed(double speed) {
		double realSpeed = speed;
		realSpeed = mReverseSpeed ? -realSpeed : realSpeed;
		
		if (RunConstants.SPEED_PID) {
			double rps = realSpeed;
			if (mFastMode)
			{
				rps *= DriveConstants.MAX_RPS_FAST_MODE;
			}
			else
			{
				rps *= DriveConstants.MAX_RPS_SLOW_MODE;
			}
			
			double rawSpeedUnits = rps / DriveConstants.RAW_TO_RPS;
			
			mDrive.set(rawSpeedUnits);
		}
		else {
			double maxSpeed = 1.0;
			if (realSpeed > maxSpeed)
			{
				realSpeed = maxSpeed;
			}
			if (realSpeed < -maxSpeed)
			{
				realSpeed = -maxSpeed;
			}

			mDrive.set(realSpeed);
		}
	}
}
