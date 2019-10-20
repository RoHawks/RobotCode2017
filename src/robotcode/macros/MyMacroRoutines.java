package robotcode.macros;

import org.usfirst.frc.team3419.robot.Robot;

import constants.RunConstants;
import constants.UltrasonicAlignerConstants;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.XboxController;
import resource.Vector;
import robotcode.GearGrabber;
import robotcode.PoseEstimator;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.SpeedMode;

public class MyMacroRoutines {
	public static Macro ULTRASONIC_PLACEMENT (
			DriveTrain pDriveTrain, 
			GearGrabber pGearGrabber,
			Ultrasonic pUltrasonic,
			double pTolerance)
	{
		return new MacroRoutine (new Macro[] {
				//(new SetWheelDirectionMacro(pDriveTrain, 0, DriveTrain.LEFT)),
				(new SetWheelDirectionMacro (pDriveTrain, 0, Vector.createPolar(90, 1.0))),
				(new AlignLR_Ultrasonic(pDriveTrain, pUltrasonic, pTolerance)),
				(new SetWheelDirectionMacro(pDriveTrain, 0, Vector.createPolar(0, 1.0))),
//				(new LinearMotionMacro(pDriveTrain, 1750, Vector.createPolar(180, 0.2), 500, false, true, false, SpeedMode.SLOW)),
//				(new GearMacro.LiftFrame(pGearGrabber)),
//				(new LinearMotionMacro(pDriveTrain, 1750, Vector.createPolar(0, 0.2), 500, false, true, false, SpeedMode.SLOW)),
				(new LinearMotionMacro(pDriveTrain, 1000, Vector.createPolar(180, 0.4), 200, false, true, false, SpeedMode.SLOW)),
				(new GearMacro.LiftFrame(pGearGrabber)),
				(new LinearMotionMacro(pDriveTrain, 1200, Vector.createPolar(0, 0.4), 200, false, true, false, SpeedMode.SLOW)),
				(new GearMacro.ReleaseGear(pGearGrabber)),
				(new LinearMotionMacro(pDriveTrain, 1000, Vector.createPolar(180, 0.8), 500, true, true, false, SpeedMode.SLOW)),
				(new GearMacro.LowerFrame(pGearGrabber)),
		});
	}

	public static Macro CAMERA_PLACEMENT (
			DriveTrain pDriveTrain, 
			GearGrabber pGearGrabber,
			double pPegAngle)
	{
		return new MacroRoutine (new Macro[] {
				(new RotateToAngleMacro(pDriveTrain, pPegAngle, 2, 200)),
				(new SetWheelDirectionMacro(pDriveTrain, 0, DriveTrain.FORWARD)),
				(new AlignPegCamera(pDriveTrain)),
				(new GearMacro.LiftFrame(pGearGrabber)),
				(new LinearMotionMacro(pDriveTrain, 1000, Vector.createPolar(0, 0.5), 500, true, true, false, SpeedMode.SLOW)),
				(new GearMacro.ReleaseGear(pGearGrabber)),
				(new LinearMotionMacro(pDriveTrain, 2000, Vector.createPolar(180, 0.5), 500, true, true, false, SpeedMode.SLOW)),
				(new GearMacro.LowerFrame(pGearGrabber)),
		});
	}

	public static Macro MID_STATION_ULTRASONIC (
			DriveTrain pDriveTrain,
			GearGrabber pGearGrabber,
			Ultrasonic pUltrasonic)
	{
		return new MacroRoutine (new Macro[] {
			(new SetWheelDirectionMacro(pDriveTrain, 0, Vector.createPolar(0, 1.0))),
			//(new LinearMotionMacro(pDriveTrain, 3100, Vector.createPolar(0, 0.4), 1000, true, true, false, SpeedMode.FAST)),
			(new LinearMotionMacro(pDriveTrain, 1850, Vector.createPolar(0, 0.8 /*0.72*/), 500, true, true, false, SpeedMode.SLOW)),
			(new LinearMotionMacro(pDriveTrain, 200, Vector.createPolar(180, 0.3 /*0.27*/), 50, true, true, false, SpeedMode.SLOW)),
			//(new SetWheelDirectionMacro(pDriveTrain, 0, Vector.createPolar(90, 1.0))),
			(ULTRASONIC_PLACEMENT(pDriveTrain, pGearGrabber, pUltrasonic, UltrasonicAlignerConstants.TOLERANCE_MIDDLE))
		});
	}
	
	public static Macro LEFT_STATION_ULTRASONIC (
			DriveTrain pDriveTrain,
			GearGrabber pGearGrabber,
			Ultrasonic pUltrasonic)
	{
		return new MacroRoutine (new Macro[] {
			(new SetWheelDirectionMacro(pDriveTrain, 0, Vector.createPolar(0, 1.0))),	
			(new LinearMotionMacro(pDriveTrain, 1675, Vector.createPolar(0, 0.9 /*0.72*/), 200, true, true, false, SpeedMode.SLOW)),
			(new RotateToAngleMacro(pDriveTrain, 300, 3, 200)),
			(new LinearMotionMacro(pDriveTrain, 850, Vector.createPolar(0, 0.6 /*0.54*/), 200, true, true, false, SpeedMode.SLOW)),
			(ULTRASONIC_PLACEMENT(pDriveTrain, pGearGrabber, pUltrasonic, UltrasonicAlignerConstants.TOLERANCE_SIDE))
		});
	}
	
	public static Macro RIGHT_STATION_ULTRASONIC (
			DriveTrain pDriveTrain,
			GearGrabber pGearGrabber,
			Ultrasonic pUltrasonic)
	{
		return new MacroRoutine (new Macro[] {
				(new LinearMotionMacro(pDriveTrain, 1675, Vector.createPolar(0, 0.9 /*0.72*/), 200, true, true, false, SpeedMode.SLOW)),
				(new RotateToAngleMacro(pDriveTrain, 60, 3, 200)),
				(new LinearMotionMacro(pDriveTrain, 850, Vector.createPolar(0, 0.6 /*0.54*/), 200, true, true, false, SpeedMode.SLOW)),
				(ULTRASONIC_PLACEMENT(pDriveTrain, pGearGrabber, pUltrasonic, UltrasonicAlignerConstants.TOLERANCE_SIDE))
		});
	}
	
	public static Macro DRIVE_STRAIGHT (
			DriveTrain pDriveTrain)
	{
		return new MacroRoutine (new Macro[] {
			(new SetWheelDirectionMacro(pDriveTrain, 0, Vector.createPolar(0, 1.0))),
			(new LinearMotionMacro(pDriveTrain, 3500, Vector.createPolar(0, 0.4 /*0.36*/), 1000, true, true, false, SpeedMode.FAST)),
		});
	}
	
	public static Macro DRIVE_STRAIGHT_TURN_LEFT_PEG (
			DriveTrain pDriveTrain)
	{
		return new MacroRoutine (new Macro[] {
			(new SetWheelDirectionMacro(pDriveTrain, 0, Vector.createPolar(0, 1.0))),
			(new LinearMotionMacro(pDriveTrain, 2500, Vector.createPolar(0, 0.8 /*0.72*/), 500, true, true, false, SpeedMode.SLOW)),
			(new RotateToAngleMacro(pDriveTrain, 300, 3, 200)),
		});
	}
	
	public static Macro DRIVE_STRAIGHT_TURN_RIGHT_PEG (
			DriveTrain pDriveTrain)
	{
		return new MacroRoutine (new Macro[] {
			(new SetWheelDirectionMacro(pDriveTrain, 0, Vector.createPolar(0, 1.0))),
			(new LinearMotionMacro(pDriveTrain, 2500, Vector.createPolar(0, 0.8 /*0.72*/), 500, true, true, false, SpeedMode.SLOW)),
			(new RotateToAngleMacro(pDriveTrain, 60, 3, 200)),
		});
	}
	
	public static Macro RELEASE_AND_DRIVE (
			DriveTrain pDriveTrain,
			GearGrabber pGearGrabber)
	{
		return new MacroRoutine (new Macro[] {
				(new GearMacro.ReleaseGear(pGearGrabber)),
				(new DelayMacro(150)),
				(new LinearMotionMacro(pDriveTrain, 1000, Vector.createPolar (180, 0.6), 500, true, false, false, SpeedMode.FAST)),
				(new GearMacro.LowerFrame(pGearGrabber)),
				(new RotateToAngleMacro(pDriveTrain, 0, 4, 100)),
				});
	}
	
	public static Macro NOTHING_MACRO ()
	{
		return new Macro() {
			
			@Override
			public void runFrame() {
				
			}
			
			@Override
			public boolean isComplete() {
				return true;
			}
		};
	}
	
	public static Macro ALIGN_PEG_ZED (
			DriveTrain pDriveTrain, 
			GearGrabber pGearGrabber,
			PoseEstimator pPoseEstimator, 
			XboxController pXbox,
			Robot pRobot)
	{
		return new MacroRoutine (new Macro[] {
				(new GearMacro.LiftFrame(pGearGrabber)),
				(new AlignPegZedV2 (pDriveTrain, pPoseEstimator, pXbox)),
				RELEASE_AND_DRIVE(pDriveTrain, pGearGrabber)},
			((Robot r) -> {
				r.mDriveTrain.setSlowStick (true); 
				r.mDriveTrain.setPOV(true);
				pPoseEstimator.resetCount();
			}),
			((Robot r) -> {
				r.mDriveTrain.setSlowStick (false); 
				r.mDriveTrain.setPOV (RunConstants.DRIVE_DOWN_FIELD_POV);
			}),
			pRobot); 
	}
	
	public static Macro ALIGN_ANGLE_ZED (
			DriveTrain pDriveTrain, 
			GearGrabber pGearGrabber,
			PoseEstimator pPoseEstimator, 
			Robot pRobot)
	{
		return new MacroRoutine (new Macro[] {
				(new GearMacro.LiftFrame(pGearGrabber)),
				(new AlignZedAngle(pDriveTrain, pPoseEstimator))},
				((Robot r) -> {
					r.mDriveTrain.setSlowStick (true); 
					r.mDriveTrain.setPOV(true);
					pPoseEstimator.resetCount();
				}),
				((Robot r) -> {}),
				pRobot);
	}
	
	public static Macro ALIGN_ANGLE_LR_ZED (
			DriveTrain pDriveTrain, 
			GearGrabber pGearGrabber,
			PoseEstimator pPoseEstimator, 
			Robot pRobot)
	{
		return new MacroRoutine (new Macro[] {
				(new GearMacro.LiftFrame(pGearGrabber)),
				(new AlignPegZedAngleLR(pDriveTrain, pPoseEstimator))},
				((Robot r) -> {
					r.mDriveTrain.setSlowStick (true); 
					r.mDriveTrain.setPOV(true);
					pPoseEstimator.resetCount();
				}),
				((Robot r) -> {}),
				pRobot);
	}

}
