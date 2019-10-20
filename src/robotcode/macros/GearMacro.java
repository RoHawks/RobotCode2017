package robotcode.macros;

import robotcode.GearGrabber;

public abstract class GearMacro extends Macro {
	protected GearGrabber mGearGrabber;
	
	public GearMacro (GearGrabber pGearGrabber)
	{
		mGearGrabber = pGearGrabber;
	}
	
	public static class LiftFrame extends GearMacro
	{

		public LiftFrame(GearGrabber pGearGrabber) {
			super(pGearGrabber);
		}
		
		@Override
		public void runFrame() {
			mGearGrabber.setGearUp(true);
		}

		@Override
		public boolean isComplete() {
			return mGearGrabber.getGearUp();
		}
	}
	
	public static class LowerFrame extends GearMacro
	{

		public LowerFrame(GearGrabber pGearGrabber) {
			super(pGearGrabber);
		}

		@Override
		public void runFrame()
		{
			mGearGrabber.setShouldDown(true);
		}

		@Override
		public boolean isComplete() {
			return mGearGrabber.getGearDown();
		}
	}
	
	public static class ReleaseGear extends GearMacro
	{
		public ReleaseGear(GearGrabber pGearGrabber) {
			super(pGearGrabber);
		}

		@Override
		public void runFrame() {
			mGearGrabber.setReleaseGear(true);
		}

		@Override
		public boolean isComplete() {
			return mGearGrabber.getGearReleased();
		}
		
	}
}
