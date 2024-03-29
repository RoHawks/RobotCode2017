package robotcode;

import edu.wpi.first.wpilibj.DigitalOutput;
import resource.Color;


/**
 * Representation of a LPD8806 based led strip.
 *
 * @author Gert Leenders
 */
public final class LedStrip {

    private static final int GAMMA_LENGTH = 256;
    private static final byte[] GAMMA = new byte[GAMMA_LENGTH];
    private int numberOfLeds;
    private RGBLed[] ledBuffer;
    private double brightness;
    private boolean suspendUpdates = false;
    private DigitalOutput mData;
    private DigitalOutput mClock;
    
    static {
        for (int i = 0; i < GAMMA_LENGTH; i++) {
            int j = (int) (Math.pow(((double) i) / 255.0, 2.5) * 127.0 + 0.5);
            GAMMA[i] = (byte) (0x80 | j);
        }
    }
    
    /**
     * Initialize a led strip.
     *
     * @param numberOfLeds the number of leds on the strip
     * @param brightness   the overall brightness of the leds
     * @throws IllegalArgumentException
     */
    public LedStrip(DigitalOutput pData, DigitalOutput pClock,
    		final int numberOfLeds, final double brightness) throws IllegalArgumentException {
        if (brightness < 0 || brightness > 1.0) {
            throw new IllegalArgumentException("Brightness must be between 0.0 and 1.0");
        }
        this.numberOfLeds = numberOfLeds;
        this.ledBuffer = new RGBLed[numberOfLeds];
        for (int i = 0; i < numberOfLeds; i++) {
            ledBuffer[i] = new RGBLed();
        }

        this.brightness = brightness;
        this.mData = pData;
        this.mClock = pClock;
        startBitbang();
    }

    /**
     * @param suspendUpdates if true, the trip wil ignore updates
     */
    public void setSuspendUpdates(boolean suspendUpdates) {
        this.suspendUpdates = suspendUpdates;
    }

    /**
     * Set all leds off.
     */
    public void allOff() {
        fill(0, 0, 0);
        update();
    }

    /**
     * Fill all leds with a specified color.
     *
     * @param red   value between 0 and 255 for the red led
     * @param green value between 0 and 255 for the green led
     * @param blue  value between 0 and 255 for the blue led
     */
    public void fill(final int red, final int green, final int blue) {
        fill(red, green, blue, 1, numberOfLeds);
    }

    /**
     * Fill all leds with a specified color and set the overall brightness.
     *
     * @param red        value between 0 and 255 for the red led
     * @param green      value between 0 and 255 for the green led
     * @param blue       value between 0 and 255 for the blue led
     * @param brightness value between 0 and 1 for the brightness
     */
    public void fill(final int red, final int green, final int blue, final double brightness) {
        fill(red, green, blue, 1, numberOfLeds, brightness);
    }

    /**
     * Fill a part of the led strip with a specified color.
     *
     * @param red   value between 0 and 255 for the red led
     * @param green value between 0 and 255 for the green led
     * @param blue  value between 0 and 255 for the blue led
     * @param start the start led position in the led strip
     * @param end   the end led position in the led strip
     * @throws IllegalArgumentException
     */
    public void fill(final int red, final int green, final int blue, final int start, final int end) throws IllegalArgumentException {
        fill(red, green, blue, start, end, brightness);
    }

    /**
     * Fill a part of the led strip with a specified color and set the brightness.
     *
     * @param red        value between 0 and 255 for the red led
     * @param green      value between 0 and 255 for the green led
     * @param blue       value between 0 and 255 for the blue led
     * @param start      the start led position in the led strip
     * @param end        the end led position in the led strip
     * @param brightness value between 0 and 1 for the brightness
     * @throws IllegalArgumentException
     */
    public void fill(final int red, final int green, final int blue, final int start, final int end,
                     final double brightness) throws IllegalArgumentException {

        if (red < 0 || green < 0 || blue < 0 || red > 255 || green > 255 || blue > 255) {
            throw new IllegalArgumentException("Red, green and blue values must be between 0 and 255.");
        }

        if (start < 1 || end > (numberOfLeds + 1)) {
            throw new IllegalArgumentException("Led start must be greater then 0, end must be smaller then " + (numberOfLeds + 1) + ".");
        }

        if (end < start) {
            throw new IllegalArgumentException("End must be greater then or equal as start.");
        }

        for (int i = start; i <= end; i++) {
            setLed(i, red, green, blue, brightness);
        }
    }

    /**
     * Set the color of an individual led.
     *
     * @param number the number of the led in the led strip
     * @param red    value between 0 and 255 for the red led
     * @param green  value between 0 and 255 for the green led
     * @param blue   value between 0 and 255 for the blue led
     */
    public void setLed(final int number, final int red, final int green, final int blue) {
        setLed(number, red, green, blue, brightness);
    }

    /**
     * Switch a led off.
     *
     * @param number the number of the led in the led strip
     */
    public void setLedOff(final int number) {
        setLed(number, 0, 0, 0, 0);
    }

    /**
     * Set the color and brightness of an individual led.
     *
     * @param number     the number of the led in the led strip
     * @param red        value between 0 and 255 for the red led
     * @param green      value between 0 and 255 for the green led
     * @param blue       value between 0 and 255 for the blue led
     * @param brightness value between 0 and 1 for the brightness
     */
    public void setLed(final int number, final int red, final int green, final int blue, final double brightness) {
        if (number < 1 || number > numberOfLeds) {
            throw new IllegalArgumentException("led number must be greater then 0 and smaller then " + (numberOfLeds + 1) + ".");
        }

        ledBuffer[number - 1].set(red, green, blue, brightness);
    }

    /**
     * Update the strip in order to show its new settings.
     */
    public void update() {
        if (suspendUpdates) {
            return;
        }

        final byte packet[] = new byte[numberOfLeds * 3];

        //POTENTIALLY CHANGE THIS IF BGRA
        for (int i = 0; i < numberOfLeds; i++) {
            packet[i * 3] = ledBuffer[i].getBlue();
            packet[(i * 3) + 1] = ledBuffer[i].getRed();
            packet[(i * 3) + 2] = ledBuffer[i].getGreen();
        }

        
        sendPacket(this.numberOfLeds * 3, packet);
        byte endPacket[] = {(byte) 0x00};
        sendPacket(1, endPacket);
    }

    
    public void sendPacket(int size, byte[] packet) {
    	int p, bit;
        for(int i = size - 1; i >= 0; i--)
        {
        	p = packet[i];
        	for(bit = 0x80; bit > 0; bit >>= 1) {
        		if((p & bit) != 0) mData.set(true);
        		else mData.set(false);
        		mClock.set(true);
        		mClock.set(false);
        	}
        }
    }
    
    /**
     * Simple test function to test your led strip.
     *
     * @throws InterruptedException
     */
    public void testStrip() throws InterruptedException {
        allOff();

        fill(0, 255, 0);
        update();

        Thread.sleep(2000);

        fill(0, 0, 255);
        update();

        Thread.sleep(2000);

        fill(255, 0, 0);
        update();

        Thread.sleep(2000);

        allOff();
    }

    /**
     * RGBLed represents a 'single' led on a led strip.
     * In reality these 'single' leds consist out of 3 leds, a red, a green and a blue one.
     *
     * @author Gert Leenders
     */
    private class RGBLed {
        private byte red;
        private byte green;
        private byte blue;

        /**
         * Initiate a single led in a led strip.
         *
         * @param red        value between 0 and 255 for the red led
         * @param green      value between 0 and 255 for the green led
         * @param blue       value between 0 and 255 for the blue led
         * @param brightness overall brightness for the led combination
         */
        public void set(final int red, final int green, final int blue, final double brightness) {
            this.red   = GAMMA[(int) (red * brightness)];
            this.green = GAMMA[(int) (green * brightness)];
            this.blue  = GAMMA[(int) (blue * brightness)];
        }

        /**
         * @return the value for the green led (between 0 and 255)
         */
        public byte getGreen() {
            return green;
        }

        /**
         * @return the value for the blue led (between 0 and 255)
         */
        public byte getBlue() {
            return blue;
        }

        /**
         * @return the value for the red led (between 0 and 255)
         */
        public byte getRed() {
            return red;
        }
    }
    
    public void startBitbang() {
    	mData.set(false);
    	for(int i=((numberOfLeds+31)/32)*8; i>0; i--) {
    		mClock.set(true);
    		mClock.set(false);
    	}
    }
    
    public int numLeds() {
    	return numberOfLeds;
    }
    
    public void setColor(Color col, int led) {
		if(led < 1 || led > numLeds()) return;
		setLed(led, col.getRed(), col.getGreen(), col.getBlue(), col.getBrightness());
	}
	
	public void setColor(Color col, int led, double brightness) {
		setColor(new Color(col.getRed(), col.getGreen(), col.getBlue(), brightness), led);
	}
	
	public void fillColor(Color col) {
		fill(col.getRed(), col.getGreen(), col.getBlue(), col.getBrightness());
	}
	
	public void set(Color[] cols) {
		for(int i = 0; i < numberOfLeds; i++) {
			setColor(cols[i], i+1);
		}
	}
}