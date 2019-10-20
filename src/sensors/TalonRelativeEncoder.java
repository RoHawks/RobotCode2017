package sensors;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class TalonRelativeEncoder extends RelativeEncoder {
	private CANTalon mTalon;
	private boolean mReversed;
	
	public TalonRelativeEncoder (CANTalon pTalon, boolean pReversed, double pTicksToRPM) {
		super(pTicksToRPM);
		mTalon = pTalon;
		mReversed = pReversed;
		
		mTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		mTalon.reverseSensor(mReversed);
	}

	@Override
	public double getRawTicksPerSecond() {
		return mTalon.getSpeed();
	}
}
