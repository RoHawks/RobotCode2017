package robotcode;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class SingleSolenoidReal implements SolenoidInterface {
	private Solenoid mSingleSolenoid;
	
	public SingleSolenoidReal(int pPort) {
		mSingleSolenoid = new Solenoid (pPort);
	}
	
	@Override
	public void set(Value pDirection) { 
		//forward maps to true, backward maps to false
		mSingleSolenoid.set(pDirection == Value.kForward);
	}

	@Override
	public Value get() {
		return mSingleSolenoid.get() ? Value.kForward : Value.kReverse;
	}

}
