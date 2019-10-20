 package sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;

public class RobotAngle extends RotationInputter implements PIDSource {
	AHRS mNavX;
	boolean mReversed;
	
	public RobotAngle (AHRS pNavX, boolean pReversed, double pOffset) {
		super (pReversed, pOffset);
		mReversed = pReversed;
		mNavX = pNavX;
	}
	
	public double getRawAngleDegrees() {
		return mNavX.getAngle();
	}
	
	public double getAngularVelocity() {
		return Math.toDegrees ((mReversed ? -1 : 1) * mNavX.getRate());
	}

	public double pidGet() {
		return getAngleDegrees();
	}	
	
	public void reset() {
		mNavX.reset();
	}
}
