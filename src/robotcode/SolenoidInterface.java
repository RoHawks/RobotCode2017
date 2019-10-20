package robotcode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface SolenoidInterface {
	public void set (Value pDirection);
	public Value get ();
}
