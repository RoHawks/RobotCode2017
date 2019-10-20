package robotcode.macros;

import constants.DriveConstants;
import constants.RunConstants;
import constants.ZedAlignmentConstants;
import edu.wpi.first.wpilibj.LocalPIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import robotcode.PoseEstimator;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;
import robotcode.driving.DriveTrain.SwerveMovement;
import robotcode.driving.DriveTrain.SwerveMovement.SwerveState;
import robotcode.pid.GenericPIDOutput;
import robotcode.pid.PersistentlyInRange;

public class AlignZedAngle extends DriveMacro {

	private PoseEstimator mPoseEstimator;
	private PIDSource mAngleInput;
	private GenericPIDOutput mAngleOutput;
	private LocalPIDController mAnglePID;

	
	public AlignZedAngle(
			DriveTrain pDriveTrain, 
			PoseEstimator pPoseEstimator) {
		
		super(pDriveTrain);
		mPoseEstimator = pPoseEstimator;
	}
	
	public void start()
	{	


		mAngleInput = new PIDSource() {
			public double pidGet() {
				return ResourceFunctions.putAngleInRange(mPoseEstimator.getAngle());
			}

			public PIDSourceType getPIDSourceType() {return null;}
			public void setPIDSourceType(PIDSourceType pidSource) {}
		};
		

		mAngleOutput = new GenericPIDOutput();

		
		
		mAnglePID = new LocalPIDController(
				0.02, 0.0, 0.0,
				mAngleInput, mAngleOutput);
		
		mAnglePID.setOutputRange(
				-0.2, 0.2);
		
		
		mAnglePID.setInputRange(0, 360);
		mAnglePID.setContinuous(true);
		mAnglePID.setSetpoint (ZedAlignmentConstants.ANGLE_SETPOINT);
		mAnglePID.enable();	
	}

	public SwerveMovement getMovement ()
	{
		boolean rotating = false;
		double angularVel = 0.0;
		if (mPoseEstimator.getValid())
		{
			rotating = true;
			angularVel = mAnglePID.get();
		}
		else
		{
			rotating = false;
		}
		
		if (Math.abs(angularVel) < 0.05)
		{
			rotating = false;
		}
		
		if (rotating)
		{
			return new SwerveMovement (angularVel, Vector.createPolar(0.0, 0.0), SwerveState.NORMAL, SpeedMode.SLOW);
		}
		else
		{
			return new SwerveMovement (1.0, Vector.createPolar(0.0, 0.0), SwerveState.STAY_STILL, SpeedMode.SLOW);
		}
	}

	public boolean isComplete() {
		return false;
	}

}
