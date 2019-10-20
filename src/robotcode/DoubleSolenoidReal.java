package robotcode;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoidReal implements SolenoidInterface {
	private DoubleSolenoid mDoubleSolenoid;
	
	public DoubleSolenoidReal (int pInPort, int pOutPort) 
	{
		mDoubleSolenoid = new DoubleSolenoid (pInPort, pOutPort);
	}
	
	@Override
	public void set(Value pDirection) {
		mDoubleSolenoid.set (pDirection);
	}

	@Override
	public Value get() {
		return mDoubleSolenoid.get();
	}

}
