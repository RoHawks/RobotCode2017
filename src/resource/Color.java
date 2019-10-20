package resource;

public class Color {
	private int mRed;
	private int mGreen;
	private int mBlue;
	private double mBrightness;
	
	public static Color
	RED = new Color (255, 0, 0),
	GREEN = new Color (0, 255, 0),
	BLUE = new Color (0, 0, 255),
	PURPLE = new Color (200, 50, 150),
	OTHER_PURPLE = new Color (255, 100, 255),
	TURQUOISE = new Color (65, 225, 200),
	YELLOW = new Color (255, 255, 0);
	
	
	public Color(int r, int g, int b, double br) {
		mRed = r;
		mGreen = g;
		mBlue = b;
		mBrightness = br;
	}
	
	public Color(int r, int g, int b) {
		this(r, g, b, 1);
	}
	
	public int getRed() {
		return mRed;
	}
	
	public int getGreen() {
		return mGreen;
	}
	
	public int getBlue() {
		return mBlue;
	}
	
	public double getBrightness() {
		return mBrightness;
	}
}