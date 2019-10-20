package sensors;

public class DummyRelativeEncoder extends RelativeEncoder {

	public DummyRelativeEncoder(double pTicksToRPM) {
		super(pTicksToRPM);
	}

	@Override
	public double getRawTicksPerSecond() {
		return 0;
	}

}
