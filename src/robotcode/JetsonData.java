package robotcode;

import constants.CamAlignerConstants;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class JetsonData {
	private static NetworkTable mJetsonTable;
	
	public static void setNetworkTable (NetworkTable pJetsonTable)
	{
		mJetsonTable = pJetsonTable;
	}
	
	public static boolean jetsonSeesTarget() {
		return mJetsonTable.getBoolean(CamAlignerConstants.IS_VALID_KEY, false); //default to not seeing the target
	}

	/**
	 * determines LR offset of the target by averaging corner x-values
	 * @return LR offset of vision targets
	 */
	public static double jetsonLR() {
		//return average X position
		double ul_x, ur_x, ll_x, lr_x;
		ul_x = mJetsonTable.getNumber(CamAlignerConstants.UL_X_KEY, CamAlignerConstants.LR_SETPOINT);
		ur_x = mJetsonTable.getNumber(CamAlignerConstants.UR_X_KEY, CamAlignerConstants.LR_SETPOINT);
		ll_x = mJetsonTable.getNumber(CamAlignerConstants.LL_X_KEY, CamAlignerConstants.LR_SETPOINT);
		lr_x = mJetsonTable.getNumber(CamAlignerConstants.LR_X_KEY, CamAlignerConstants.LR_SETPOINT);

		double average = ((ul_x + ur_x + ll_x + lr_x) / 4.0);
		return average;
	}

	/**
	 * determines value with the correct sign monotonic to angle offset of the target by subtracting the left height from the right height
	 * @return proxy for angle offset
	 */
	public static double jetsonAngle() {
		//return left height - right height
		double ul_y, ur_y, ll_y, lr_y;
		ul_y = mJetsonTable.getNumber(CamAlignerConstants.UL_Y_KEY, 0.5);
		ur_y = mJetsonTable.getNumber(CamAlignerConstants.UR_Y_KEY, 0.5);
		ll_y = mJetsonTable.getNumber(CamAlignerConstants.LL_Y_KEY, 0.5);
		lr_y = mJetsonTable.getNumber(CamAlignerConstants.LR_Y_KEY, 0.5);

		double leftHeight = ll_y - ul_y;
		double rightHeight = lr_y - ur_y;
		return rightHeight - leftHeight;
	}


	/**
	 * determines value monotonic to the distance away by reciprocal of width
	 * @return proxy for distance
	 */
	public static double jetsonDistance() 
	{
		//return average X position
		double ul_x, ur_x, ll_x, lr_x;
		ul_x = mJetsonTable.getNumber(CamAlignerConstants.UL_X_KEY, 0.0);
		ur_x = mJetsonTable.getNumber(CamAlignerConstants.UR_X_KEY, 1.0);
		ll_x = mJetsonTable.getNumber(CamAlignerConstants.LL_X_KEY, 0.0);
		lr_x = mJetsonTable.getNumber(CamAlignerConstants.LR_X_KEY, 1.0);

		double leftX = (ul_x + ll_x) / 2;
		double rightX = (ur_x + lr_x) / 2;
		return 1.0 / (rightX - leftX);
	}
}
