package org.usfirst.frc.team3419.robot;

import edu.wpi.first.wpilibj.SampleRobot;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import java.util.function.Predicate;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import com.kauailabs.navx.frc.AHRS;

import constants.CamAlignerConstants;
import constants.Configurables;
import constants.ControlPorts;
import constants.AutonomousConstants.AutoMode;
import constants.DriveConstants;
import constants.GrabberConstants;
import constants.LEDConstants;
import constants.Ports;
import constants.PressureConstants;
import constants.RunConstants;
import constants.UltrasonicAlignerConstants;
import constants.RunConstants.RunMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.Color;
import resource.ResourceFunctions;
import robotcode.Climber;
import robotcode.DisplacementEstimator;
import robotcode.DoubleSolenoidReal;
import robotcode.GearGrabber;
import robotcode.JetsonData;
import robotcode.JetsonTX1UDP_Client;
import robotcode.PoseEstimator;
import robotcode.GearGrabber.GearState;
import robotcode.LedStrip;
import robotcode.SingleSolenoidReal;
import robotcode.SolenoidInterface;
import robotcode.VelocityEstimator;
import sensors.RotationInputter;
import sensors.DummyRelativeEncoder;
import sensors.RelativeEncoder;
import sensors.RobotAngle;
import sensors.TalonAbsoluteEncoder;
import sensors.TalonRelativeEncoder;
import sensors.USDigitalMA3Encoder;
import robotcode.driving.*;
import robotcode.macros.AlignPegZedV2;
import robotcode.macros.Macro;
import robotcode.macros.MacroRoutine;
import robotcode.macros.MyMacroRoutines;

public class Robot extends SampleRobot {

	//exclusively final robot objects
	private RelativeEncoder[] mDriveEncoders;
	private SolenoidInterface mShifterSolenoid;

	private DigitalOutput mLED_Data, mLED_Clock;
	private LedStrip mLEDStrip;

	//need to initialize these seperately because it has to belong to both the encoder and speedinputter
	private CANTalon[] mTurnCANTalons;
	private CANTalon[] mDriveCANTalons;

	//both final and prototype robot objects
	private PowerDistributionPanel mPDP;
	private Compressor mCompressor;

	private Thread mGearGrabberThread;
	private GearGrabber mGearGrabber;
	
	private Thread mPoseEstimatorThread;
	private PoseEstimator mPoseEstimator;
	private JetsonTX1UDP_Client mJetsonTX1UDP_Client;
	
	private AnalogInput mPressureSensor;
	
	private Thread mVelocityEstimatorThread;
	private VelocityEstimator mVelocityEstimator;
	
	private Thread mDisplacementEstimatorThread;
	private DisplacementEstimator mDisplacementEstimator;
	
	private long mLoopTimeMillis;

	//timing fields for logging
	private long 
	mTimeStart, 
	mCompressorTimeTotal, 
	mCompressorTimeContinuous, 
	mLastTimestampLogger;

	private DigitalInput mBreakBeam, mLimitSwitch;
	private RotationInputter[] mTurnEncoders;
	private Encoder mWinchEncoder;

	//speed controllers
	private SpeedController[] mTurnMotors;
	private SpeedController[] mDriveMotors;
	private SpeedController mGearAlignerMotor;
	private SpeedController mWinchMotor;

	//drive train objects
	private Wheel[] mWheels;
	public DriveTrain mDriveTrain;
	private AHRS mNavX;
	private RobotAngle mRobotAngle;

	private XboxController mXbox;
	private Joystick mSecondaryStick;
	private NetworkTable mJetsonTable;
	private SolenoidInterface 
	mFramePiston, 
	mSnatcherPiston, 
	mClamperPiston;

	private DigitalInput mWinchBreakbeam;
	private Climber mClimber;

	//private PegAligner mPegAligner;

	//we have both sonars so we can choose which one to use easily should we switch
	private Ultrasonic mLeftSonar, mRightSonar;
	
	private boolean mGameHasStarted, mTimerHasStarted;
	
	private Macro mAutonomousMacro;
	private Macro mReleaseAndDriveMacro;
	
	private AutoMode mAutoChoice;
	
	private boolean mEmergencyMode;
	
	private boolean mRefillingLateGame;
	/**
	 * Wrapper class around Macros so that they can be interlaced 
	 * within the normal drive code with the start() and stop() 
	 * functions called accordingly
	 */
	private static class InGameMacro
	{
		private Macro mMacro; //macro to be used
		private Predicate<Robot> mPredicate; //lambda which is true when the macro should be run
		
		public InGameMacro (Macro pMacro, Predicate<Robot> pPredicate)
		{
			mMacro = new MacroRoutine(new Macro[]{pMacro}); //have to do this for the call of stop() to work correctly
			mPredicate = pPredicate;
		}
		
		/**
		 * tests whether or not the macro should currently be run
		 * @param pRobot parameter to the predicate which determines if the macro should be run
		 * @return true if the macro should run on this frame, false otherwise
		 */
		public boolean isTrue (Robot pRobot)
		{
			return mPredicate.test(pRobot);
		}
		
		/**
		 * performs this macro's routine on a time slice, 
		 * handles starting the macro if not already started
		 */
		public void runFrame()
		{
			//start the macro if it has not already startedd
			if (!mMacro.isRunning())
			{
				mMacro.start();
			}
			if (!mMacro.isComplete() )
			{
				mMacro.runFrame();
			}
			
		}
		
		/**
		 * handles ending the macro if it should be ended, 
		 * can safely be called every update loop
		 * @param pRobot parameter to predicate, used to know if the macro should be running
		 */
		public void handleEnding(Robot pRobot)
		{
			//stop the macro if it is running and should not be
			if ( mMacro.isRunning() && !mPredicate.test(pRobot) )
			{
				mMacro.stop();
			}	
		}
	}
	
	private InGameMacro[] mInGameMacros; //array of InGameMacros and corresponding button maps
	//private Macro mKarlAlignMacro;

	/**
	 * set certain run constants based on others, e.g:
	 * disable pneumatic in testing mode
	 * disable wheel pids in testing mode
	 */
	private void fixConstants()
	{
		//no shifter or speed pid on prototype bot
		if (RunConstants.IS_PROTOTYPE_BOT) 
		{
			RunConstants.RUNNING_PNEUMATIC_SHIFTER = false;
			RunConstants.SPEED_PID = false;
		}

		//cancel wheel pids and pneumatics when testing swerve modules individually
		if (RunConstants.RUN_MODE.isTest)
		{
			RunConstants.INITIALIZE_WHEEL_PIDS = false;
			RunConstants.RUNNING_PNEUMATIC_GRABBER = false;
		}
		
		else
		{
			RunConstants.INITIALIZE_WHEEL_PIDS = true;
		}
		
		//disable pneumatics when just testing drive train
		if (RunConstants.RUN_MODE == RunMode.SWERVE_TESTER)
		{
			RunConstants.RUNNING_PNEUMATIC_GRABBER = false;
		}

		//if neither pneumatics are running, disable the compressor
		RunConstants.RUNNING_PNEUMATICS = 
				RunConstants.RUNNING_PNEUMATIC_GRABBER || 
				RunConstants.RUNNING_PNEUMATIC_SHIFTER;
		
		//set ports for priya joystick at runtime based on run flags
		if (RunConstants.USING_CUSTOM_JOYSTICK)
		{
			ControlPorts.LEFT_STATION_BUTTON = ControlPorts.Custom.LEFT_STATION_BUTTON;
			ControlPorts.MIDDLE_STATION_BUTTON = ControlPorts.Custom.MIDDLE_STATION_BUTTON;
			ControlPorts.RIGHT_STATION_BUTTON = ControlPorts.Custom.RIGHT_STATION_BUTTON;
			
			ControlPorts.CYCLE_PLACEMENT_PORT = ControlPorts.Custom.CYCLE_PLACEMENT_PORT;
			ControlPorts.FLAP_PORT = ControlPorts.Custom.FLAP_PORT;
			ControlPorts.EMERGENCY_OPEN_PORT = ControlPorts.Custom.EMERGENCY_OPEN_PORT;
			
			ControlPorts.AUTOMATIC_CLIMB_PORT = ControlPorts.Custom.AUTOMATIC_CLIMB_PORT;
			ControlPorts.MANUAL_CLIMB_PORT = ControlPorts.Custom.MANUAL_CLIMB_PORT;
			
			ControlPorts.AUTO_CYCLE_PORT = ControlPorts.Custom.AUTO_CYCLE_PORT;
		}
		
		else
		{
			ControlPorts.LEFT_STATION_BUTTON = ControlPorts.Arcade.LEFT_STATION_BUTTON;
			ControlPorts.MIDDLE_STATION_BUTTON = ControlPorts.Arcade.MIDDLE_STATION_BUTTON;
			ControlPorts.RIGHT_STATION_BUTTON = ControlPorts.Arcade.RIGHT_STATION_BUTTON;
			
			ControlPorts.CYCLE_PLACEMENT_PORT = ControlPorts.Arcade.CYCLE_PLACEMENT_PORT;
			ControlPorts.FLAP_PORT = ControlPorts.Arcade.FLAP_PORT;
			ControlPorts.EMERGENCY_OPEN_PORT = ControlPorts.Arcade.EMERGENCY_OPEN_PORT;
			
			ControlPorts.AUTOMATIC_CLIMB_PORT = ControlPorts.Arcade.AUTOMATIC_CLIMB_PORT;
			ControlPorts.MANUAL_CLIMB_PORT = ControlPorts.Arcade.MANUAL_CLIMB_PORT;
			
			ControlPorts.AUTO_CYCLE_PORT = ControlPorts.Arcade.AUTO_CYCLE_PORT;
		}
		
		if (RunConstants.RUN_MODE.isTest)
		{
			RunConstants.USING_MACROS = false;
		}
	}

	/**
	 * initialize arrays of sensors and motors for the swerve drive, 
	 * elements will be initialized later
	 */
	private void initializeArrays() 
	{
		mTurnCANTalons = new CANTalon[4];
		mDriveCANTalons = new CANTalon[4];

		mDriveEncoders = new RelativeEncoder[4];
		mTurnEncoders = new RotationInputter[4];

		mTurnMotors = new SpeedController[4];
		mDriveMotors = new SpeedController[4];
		mWheels = new Wheel[4];
	}

	/**
	 * initialize miscallaneous elements that must occur on both the real and prototype bot
	 * e.g power distribution panel, xbox controller
	 */
	private void initializeMiscBoth()
	{
		mTimeStart = System.currentTimeMillis();
		mCompressorTimeTotal = 0;
		mCompressorTimeContinuous = 0;
		mLastTimestampLogger = -1;

		mPDP = new PowerDistributionPanel();
		mXbox = new XboxController (0);
		mSecondaryStick = new Joystick (1);

		mCompressor = new Compressor();
		
		//don't need to use the compressor if no pneumatics
		if (!RunConstants.RUNNING_PNEUMATICS) 
		{
			mCompressor.stop();
		}

		if (RunConstants.USING_TK1_VISION)
		{
			mJetsonTable = NetworkTable.getTable (CamAlignerConstants.TABLE_NAME);
			JetsonData.setNetworkTable(mJetsonTable);
		}		
	}

	/**
	 * initialize elements that either are unique to the real robot or have a unique port,
	 * e.g pressure sensor
	 */
	private void initializeMiscReal()
	{
		mPressureSensor = new AnalogInput(Ports.Real.PRESSURE_SENSOR);
		
		mLED_Data = new DigitalOutput (Ports.Real.LED_DATA);
		mLED_Clock = new DigitalOutput (Ports.Real.LED_CLOCK);
		mLEDStrip = new LedStrip (
				mLED_Data, 
				mLED_Clock, 
				LEDConstants.NUM_LEDS, 
				LEDConstants.LED_BRIGHTNESS);

		mGearAlignerMotor = new CANTalon (Ports.Real.GEAR_ALIGNER);

		mNavX = new AHRS (Port.kMXP);
		mRobotAngle = new RobotAngle(mNavX, true, 0);

		mBreakBeam = new DigitalInput (Ports.Real.BREAK_BEAM);
		mLimitSwitch = new DigitalInput (Ports.Real.LIMIT_SWITCH);

		mWinchMotor = new CANTalon (Ports.Real.WINCH_MOTOR);
		mWinchEncoder = new Encoder (
				Ports.Real.WINCH_ENCODER_A,
				Ports.Real.WINCH_ENCODER_B);
		
		mWinchBreakbeam = new DigitalInput (Ports.Real.WINCH_BREAKBEAM);
		mClimber = new Climber(mWinchMotor, mSecondaryStick, mWinchEncoder, mWinchBreakbeam);

		if (RunConstants.USING_LEFT_SONAR)
		{
			mLeftSonar = new Ultrasonic (Ports.Real.PING_LEFT, Ports.Real.ECHO_LEFT, Unit.kInches);
			mLeftSonar.setAutomaticMode(true);
			mLeftSonar.setEnabled(true);
		}
		
		if (RunConstants.USING_RIGHT_SONAR)
		{
			mRightSonar = new Ultrasonic (Ports.Real.ECHO_RIGHT, Ports.Real.PING_RIGHT, Unit.kInches);
			mRightSonar.setAutomaticMode(true);
			mRightSonar.setEnabled(true);
		}
	}

	/**
	 * initialize swerve drive for the real robot,
	 * differs in that it uses speed pids, cantalons, etc
	 */
	private void initializeSwerveDriveReal() 
	{	
		//initialize module elements
		for (int i = 0; i < 4; i++) {
			mTurnCANTalons[i] = new CANTalon (Ports.Real.TURN[i]);
			mDriveCANTalons[i] = new CANTalon (Ports.Real.DRIVE[i]);
			mDriveCANTalons[i].setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			//motors
			mTurnMotors[i] = mTurnCANTalons[i];
			mDriveMotors[i] = mDriveCANTalons[i];

			mDriveMotors[i].setInverted(DriveConstants.Modules.Real.DRIVE_MOTOR_REVERSED[i]);
			mTurnMotors[i].setInverted(DriveConstants.Modules.Real.TURN_MOTOR_REVERSED[i]);

			//encoders
			if (i == 3 && RunConstants.DRIVE_ENCODER_FOR_3)
			{
				mTurnEncoders[i] = new TalonAbsoluteEncoder(
						mDriveCANTalons[i], 
						DriveConstants.Modules.Real.TURN_ENCODER_REVERSED[i], 
						DriveConstants.Modules.Real.ENCODER_OFFSETS[i]);
			}
			else
			{
				mTurnEncoders[i] = new TalonAbsoluteEncoder(
						mTurnCANTalons[i], 
						DriveConstants.Modules.Real.TURN_ENCODER_REVERSED[i], 
						DriveConstants.Modules.Real.ENCODER_OFFSETS[i]);
			}
			
			if (RunConstants.DRIVE_ENCODER_FOR_3)
			{
				mDriveEncoders[i] = new DummyRelativeEncoder(0);
			}
			else
			{
				mDriveEncoders[i] = new TalonRelativeEncoder(
						mDriveCANTalons[i],
						DriveConstants.Modules.Real.DRIVE_ENCODER_REVERSED[i], 
						DriveConstants.RAW_TO_RPS);
			}
		}

		//only initialize wheels when in swerve mode, otherwise their PIDs will activate
		if (RunConstants.INITIALIZE_WHEEL_PIDS) {
			
			for (int i = 0; i < 4; i++) {
				if (!RunConstants.SPEED_PID)
				{
					mDriveCANTalons[i].changeControlMode(TalonControlMode.PercentVbus);
					mDriveCANTalons[i].enableBrakeMode(true);
				}	
				
				mWheels[i] = 
						new Wheel(mTurnMotors[i], mDriveMotors[i], 	
								DriveConstants.PID_Constants.Real.ROTATION_P[i], 
								DriveConstants.PID_Constants.Real.ROTATION_I[i],
								DriveConstants.PID_Constants.Real.ROTATION_D[i],
								DriveConstants.PID_Constants.Real.ROTATION_IZONE[i],

								DriveConstants.PID_Constants.Real.FAST_SPEED_P[i], 
								DriveConstants.PID_Constants.Real.FAST_SPEED_I[i],
								DriveConstants.PID_Constants.Real.FAST_SPEED_D[i],
								DriveConstants.PID_Constants.Real.FAST_SPEED_F[i],
								
								DriveConstants.PID_Constants.Real.SLOW_SPEED_P[i], 
								DriveConstants.PID_Constants.Real.SLOW_SPEED_I[i],
								DriveConstants.PID_Constants.Real.SLOW_SPEED_D[i],
								DriveConstants.PID_Constants.Real.SLOW_SPEED_F[i],
								
								DriveConstants.Modules.Real.X_OFF[i],
								DriveConstants.Modules.Real.Y_OFF[i], 
								
								mTurnEncoders[i], mDriveEncoders[i]);
				
				if (RunConstants.SPEED_PID)
				{
					configureTalon (mDriveCANTalons[i], 0);
				}
			}
		}
	}	
		
	/**
	 * configures a cantalon for use in a game, sets ramp rate, maxvoltage, etc
	 * @param pTalon cantalon to be initialized
	 * @param pProfile profile for the cantalon (0 or 1)
	 */
	private void configureTalon (CANTalon pTalon, int pProfile)
	{	
		//double rampRate = 1000.0 * DriveConstants.MAX_VOLTAGE / DriveConstants.VOLTAGE_RAMP_MILLIS;
		pTalon.setProfile(pProfile);
		pTalon.enableBrakeMode(true);
		pTalon.configNominalOutputVoltage(+0.0f, -0.0f);
		pTalon.configPeakOutputVoltage(DriveConstants.MAX_VOLTAGE, -DriveConstants.MAX_VOLTAGE);
		//pTalon.setVoltageRampRate(rampRate);
		//pTalon.setVoltageRampRate(1000000);
		pTalon.setSetpoint(0);
		pTalon.setCloseLoopRampRate(0);
		pTalon.changeControlMode(TalonControlMode.Speed);
		if (!RunConstants.DRIVE_ENCODER_FOR_3)
		{
			pTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		}
		pTalon.enableControl();
		pTalon.enable();
	}	
		
	/**
	 * initialize pneumatics elements on the real robot
	 * includes shifter and grabber
	 */
	private void initializePneumaticsReal() 
	{	
		if (RunConstants.RUNNING_PNEUMATIC_SHIFTER) 
		{
			mShifterSolenoid = new DoubleSolenoidReal (
					Ports.Real.SHIFTER_SOLENOID_IN, 
					Ports.Real.SHIFTER_SOLENOID_OUT);
			mShifterSolenoid.set(DriveConstants.SLOW_SHIFT_DIR);
		}
		
		if (RunConstants.RUNNING_PNEUMATIC_GRABBER)
		{
			mFramePiston = new DoubleSolenoidReal (
					Ports.Real.FRAME_PISTON_IN, 
					Ports.Real.FRAME_PISTON_OUT);
		
			mClamperPiston = new SingleSolenoidReal (Ports.Real.CLAMPER_PISTON);
			mSnatcherPiston = new SingleSolenoidReal (Ports.Real.SNATCHER_PISTON);
		}
	}	
		
	/**
	 * initialized miscallaneous elements either unique to the practice robot or with a unique port
	 */
	private void initializeMiscPrototype()
	{	
		mGearAlignerMotor = new Talon (Ports.Prototype.GEAR_ALIGNER);
		
		mNavX = new AHRS (Port.kMXP);
		mRobotAngle = new RobotAngle(mNavX, true, 0);
		
		mBreakBeam = new DigitalInput (Ports.Prototype.BREAK_BEAM);
		mLimitSwitch = new DigitalInput (Ports.Prototype.LIMIT_SWITCH);
		
		mWinchMotor = new Talon (Ports.Prototype.WINCH_MOTOR);
		mWinchEncoder = new Encoder (
				Ports.Prototype.WINCH_ENCODER_A, 
				Ports.Prototype.WINCH_ENCODER_B);
		mWinchBreakbeam = new DigitalInput (Ports.Prototype.WINCH_BREAKBEAM);
		mClimber = new Climber(mWinchMotor, mSecondaryStick, mWinchEncoder, mWinchBreakbeam);
		
		if (RunConstants.USING_LEFT_SONAR)
		{
			mLeftSonar = new Ultrasonic (Ports.Prototype.ECHO_LEFT, Ports.Prototype.PING_LEFT);
			mLeftSonar.setAutomaticMode(true);
			mLeftSonar.setEnabled(true);
		}
		
		if (RunConstants.USING_RIGHT_SONAR)
		{
			mRightSonar = new Ultrasonic (Ports.Prototype.ECHO_RIGHT, Ports.Prototype.PING_RIGHT);
			mRightSonar.setAutomaticMode(true);
			mRightSonar.setEnabled(true);
		}
	}

	/**
	 * initialized swerve drive on the miscallaneous robot, differs from real robot in that
	 * it uses USDigitalEncoders, regular Talons, etc
	 */
	private void initializeSwerveDrivePrototype() 
	{			
		//initialize module elements
		for (int i = 0; i < 4; i++) {
			//motors
			mTurnMotors[i] = new Talon (Ports.Prototype.TURN[i]);
			mDriveMotors[i] = new Talon (Ports.Prototype.DRIVE[i]);

			mDriveMotors[i].setInverted(
					DriveConstants.Modules.Prototype.DRIVE_MOTOR_REVERSED[i]);
			mTurnMotors[i].setInverted(
					DriveConstants.Modules.Prototype.TURN_MOTOR_REVERSED[i]);

			//encoders
			mTurnEncoders[i] = new USDigitalMA3Encoder(
					Ports.Prototype.ENCODERS[i], 
					DriveConstants.Modules.Prototype.ENCODER_OFFSETS[i],
					DriveConstants.Modules.Prototype.TURN_ENCODER_REVERSED[i]);
		}

		//only initialize wheels when in swerve mode, otherwise their PIDs will activate
		if (RunConstants.INITIALIZE_WHEEL_PIDS) {
			for (int i = 0; i < 4; i++) {
				mWheels[i] = 
						new Wheel(mTurnMotors[i], mDriveMotors[i], 	
								DriveConstants.PID_Constants.Prototype.ROTATION_P[i], 
								DriveConstants.PID_Constants.Prototype.ROTATION_I[i],
								DriveConstants.PID_Constants.Prototype.ROTATION_D[i],
								DriveConstants.PID_Constants.Prototype.ROTATION_IZONE[i],

								DriveConstants.Modules.Prototype.X_OFF[i],
								DriveConstants.Modules.Prototype.Y_OFF[i], 

								mTurnEncoders[i]);
			}	
		}
	}

	/**
	 * initialize pneumatics elements (only grabber, no shifter) on prototype robot
	 */
	private void initializePneumaticsPrototype() 
	{	
		if (RunConstants.RUNNING_PNEUMATIC_GRABBER)
		{
			mFramePiston = new DoubleSolenoidReal (
					Ports.Prototype.FRAME_PISTON_IN, 
					Ports.Prototype.FRAME_PISTON_OUT);

			mClamperPiston = new DoubleSolenoidReal (
					Ports.Prototype.CLAMPER_PISTON_IN, 
					Ports.Prototype.CLAMPER_PISTON_OUT);

			mSnatcherPiston = new DoubleSolenoidReal (
					Ports.Prototype.SNATCHER_PISTON_IN, 
					Ports.Prototype.SNATCHER_PISTON_OUT);
		}
	}

	/**
	 * initialize subsystems composed of previously intialized elements,
	 * e.g geargrabber, drivetrain
	 */
	public void initializeSubsystems()
	{
		if (RunConstants.RUNNING_PNEUMATIC_GRABBER)
		{
			mGearGrabber = new GearGrabber (
					mBreakBeam, mLimitSwitch, 
					mFramePiston, mClamperPiston, mSnatcherPiston, 
					mSecondaryStick);
		}

		if (RunConstants.INITIALIZE_WHEEL_PIDS)
		{
			//mPegAligner = new PegAligner(mJetsonTable, mGearGrabber, mRobotAngle, mKarlPing);
			mDriveTrain = new DriveTrain(
					mWheels, 
					mRobotAngle, 
					mShifterSolenoid, 
					mXbox, 
					mSecondaryStick);
		}
	}
	
	public void initMacros ()
	{
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			mInGameMacros = new InGameMacro[] {
					
					(new InGameMacro(
							new AlignPegZedV2 (
							mDriveTrain, 
							mPoseEstimator, 
							mXbox), 
							(Robot r) -> mXbox.getXButton() )),
					};
		}
		else
		{
			mInGameMacros = new InGameMacro[] {
					(new InGameMacro(mReleaseAndDriveMacro, 
							(Robot r) -> mXbox.getBButton() )),
					/*(new InGameMacro (
							MyMacroRoutines.ALIGN_PEG_ZED(
									mDriveTrain,
									mGearGrabber,
									mPoseEstimator,
									mXbox,
									this),
							(Robot r) -> r.mXbox.getYButton())),*/
					(new InGameMacro (
							MyMacroRoutines.ALIGN_ANGLE_LR_ZED(
								mDriveTrain,
								mGearGrabber,
								mPoseEstimator,
								this),
							(Robot r) -> r.mXbox.getXButton())),
					(new InGameMacro (
							MyMacroRoutines.ALIGN_ANGLE_ZED(mDriveTrain, mGearGrabber, mPoseEstimator, this),
							(Robot r) -> r.mXbox.getYButton())),
//					(new InGameMacro (
//							MyMacroRoutines.ULTRASONIC_PLACEMENT(mDriveTrain, mGearGrabber, mLeftSonar, UltrasonicAlignerConstants.TOLERANCE_SIDE),
//							(Robot r) -> r.mXbox.getYButton()))
					//(new InGameMacro (new RotateToAngleMacro(mDriveTrain, 60.0, 3, 200), 
					//		(Robot r) -> r.mXbox.getXButton()))
					};
		}
	}

	public Robot() {}

	/**
	 * initializes the robot, takes into account whether this is the prototype or real robot
	 */
	public void robotInit() {
		SmartDashboard.putString ("WheelTestCommand", "EndWheelTest");
		fixConstants(); //overrides certain constants based on others
		initializeArrays();
		initializeMiscBoth();

		if (RunConstants.IS_PROTOTYPE_BOT) 
		{
			initializeMiscPrototype();
			initializeSwerveDrivePrototype();
			initializePneumaticsPrototype();
		}

		else 
		{
			initializeMiscReal();
			initializeSwerveDriveReal();
			initializePneumaticsReal();
		}

		initializeSubsystems();
		
		if (RunConstants.RUNNING_PNEUMATIC_GRABBER)
		{
			mReleaseAndDriveMacro = MyMacroRoutines.RELEASE_AND_DRIVE(mDriveTrain, mGearGrabber);
//			mKarlAlignMacro = MyMacroRoutines.ULTRASONIC_PLACEMENT(
//					mDriveTrain, 
//					mGearGrabber, 
//					RunConstants.USING_LEFT_SONAR ? mLeftSonar : mRightSonar);
		}
		
		mGameHasStarted = false;
		mTimerHasStarted = false;
		
		mAutoChoice = AutoMode.MIDDLE_PLACE_GEAR; ////AutoMode.MIDDLE_PLACE_GEAR;
		
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			mClimber.setPDP(mPDP);
		}
		
		if (!RunConstants.IS_PROTOTYPE_BOT && RunConstants.RUNNING_PNEUMATIC_GRABBER)
		{
			mGearGrabber.setSquishyWheel(mGearAlignerMotor);
		}
		
		SmartDashboard.putString("DashboardCommand", "EndRecording");
		
		StringBuilder headerString = new StringBuilder();
		for (int i = 0; i < Configurables.LOGGER_COMPONENTS.length; i++)
		{
			if (i > 0)
			{
				headerString.append(",");
			}
			headerString.append(Configurables.LOGGER_COMPONENTS[i]);
		}
		
		StringBuilder wheelTestHeaderString = new StringBuilder();
		for (int i = 0; i < Configurables.WheelTestHeader.length; i++)
		{
			if (i > 0)
			{
				wheelTestHeaderString.append(",");
			}
			wheelTestHeaderString.append(Configurables.WheelTestHeader[i]);
		}
		
		mEmergencyMode = false;
		mLoopTimeMillis = -1;
		mRefillingLateGame = false;
		
		if (RunConstants.RUNNING_VELOCITY_ESTIMATOR)
		{
			mVelocityEstimator = new VelocityEstimator(mWheels, mRobotAngle, mJetsonTX1UDP_Client);
		}
		
		if (RunConstants.RUNNING_DISPLACEMENT_ESTIMATOR)
		{
			if (RunConstants.IS_PROTOTYPE_BOT)
			{
				mDisplacementEstimator = new DisplacementEstimator(mTurnEncoders, mRobotAngle);
			}
			else
			{
				mDisplacementEstimator = new DisplacementEstimator(mDriveCANTalons, mTurnEncoders, mRobotAngle);
			}
		}
		
		if (RunConstants.USING_TX1_VISION)
		{
			mPoseEstimator = new PoseEstimator(1155, Configurables.TX1_BYTES, mRobotAngle, mDisplacementEstimator);
			
			if (RunConstants.RUNNING_VELOCITY_ESTIMATOR)
			{
				mJetsonTX1UDP_Client = new JetsonTX1UDP_Client("10.34.19.12", 1154, Configurables.TX1_BYTES);
			}
		}
		
		if (RunConstants.USING_MACROS)
		{
			initMacros();
		}
		
		SmartDashboard.putString ("beep boop bop", "boop boop");
		SmartDashboard.putString("HeaderString", headerString.toString());
		SmartDashboard.putString("WheelTestHeader", wheelTestHeaderString.toString());
	}

	/**
	 * performs autonomous routine based on what was selected in the disabled mode
	 */
	public void autonomous() 
	{
		startGame();
		
		if (!RunConstants.IS_PROTOTYPE_BOT)
		{
			for (int i = 0; i < 4; i++)
			{
				mDriveCANTalons[i].enableBrakeMode(false);
			}
		}
		
		
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			while (this.isAutonomous() && this.isEnabled())
			{
				SmartDashboard.putString("CURRENT ROBOT MODE:", "AUTONOMOUS");
				logEverything();
				Timer.delay(0.005);
			}
				
		}
		else
		{
			mAutonomousMacro.execute(
					(Robot r) -> (r.isDisabled() || r.isOperatorControl()), 
					(Robot r) -> r.logEverything(), 
					this);
		}
	}
	
	@Override
	public void test()
	{
		while (isTest() && isEnabled())
		{
			mCompressor.start();
			mDriveTrain.enactMovement(DriveTrain.WHEELS_FB);
			
			mFramePiston.set (mXbox.getBumper(Hand.kRight) ? GrabberConstants.FRAME_UP : GrabberConstants.FRAME_DOWN);
			mClamperPiston.set (mXbox.getBumper(Hand.kLeft) ? GrabberConstants.CLAMPER_CLOSED : GrabberConstants.CLAMPER_OPEN);
			mSnatcherPiston.set (mXbox.getAButton() ? GrabberConstants.SNATCHER_OPEN : GrabberConstants.SNATCHER_CLOSED);
			mShifterSolenoid.set (mXbox.getBButton() ? DriveConstants.FAST_SHIFT_DIR : DriveConstants.SLOW_SHIFT_DIR);
			SmartDashboard.putNumber ("Pressure:", getPressure());
			System.out.println("Pressure: " + getPressure());
			DriverStation.reportWarning("CHECK THE RATCHET", false);
		}
	}


	public void performWinchCode() {
		while (isOperatorControl() && isEnabled()) {
			//handle subsytems
			mDriveTrain.drive();

			double speed = mXbox.getTriggerAxis(Hand.kRight) - mXbox.getTriggerAxis(Hand.kLeft);
			mWinchMotor.set(speed);

			Timer.delay(0.005);
		}
	}

	public void testAutonomous()
	{
		
		Macro myRoutine = MyMacroRoutines.MID_STATION_ULTRASONIC(
				mDriveTrain, 
				mGearGrabber, 
				RunConstants.USING_LEFT_SONAR ? mLeftSonar : mRightSonar);
		//Macro myRoutine = MyMacroRoutines.ULTRASONIC_PLACEMENT(mDriveTrain, mGearGrabber, mKarlPing);
		//Macro myRoutine = new AlignLR_Ultrasonic(mDriveTrain, mKarlPing);
		boolean readyToRun = true;
		while (isOperatorControl() && isEnabled())
		{
			if (mXbox.getAButton() && readyToRun)
			{
				long timeBegin = System.currentTimeMillis();
				myRoutine.execute((Robot r) -> mXbox.getBButton(), this);
				long timeAlign = System.currentTimeMillis() - timeBegin;
				SmartDashboard.putNumber ("Time To Align:", timeAlign);
				readyToRun = false;
			}
			
//			if (mSecondaryStick.getRawButton(8))
//			{
//				mRobotAngle.reset();
//			}
			
			if (!mXbox.getAButton())
			{
				readyToRun = true;
			}
				
			mDriveTrain.drive();
			Timer.delay(0.005);
		}
	}
	
	public void handleLeds (GearState pGearState)
	{
		Color ledColor = new Color (255, 255, 255);
		switch (pGearState)
		{
		case NONE:
			ledColor = Color.PURPLE;
			break;
		case BEGIN_STATE:
			ledColor = Color.TURQUOISE;
			break;
		case EMERGENCY_UP:
			ledColor = Color.RED;
			break;
		case FLAPPING_IN:
			ledColor = Color.OTHER_PURPLE;
			break;
		case FLAPPING_OUT:
			ledColor = Color.OTHER_PURPLE;
			break;
		case GRABBING:
			ledColor = Color.YELLOW;
			break;
		case GRABBED:
			ledColor = Color.GREEN;
			break;
		case FRAME_UP:
			ledColor = Color.GREEN;
			break;
		case READY_TO_DOWN:
			ledColor = Color.TURQUOISE;
			break;
		case READY_TO_RELEASE:
			ledColor = Color.TURQUOISE;
			break;
		case RELEASED:
			ledColor = Color.TURQUOISE;
			break;
		}
		
		if (!RunConstants.IS_PROTOTYPE_BOT)
		{
			mLEDStrip.fillColor(ledColor);
			mLEDStrip.update();
		}
	}
	
	public void logWheelSpeeds ()
	{
		if (RunConstants.SPEED_PID && !RunConstants.IS_PROTOTYPE_BOT)
		{
			for (int i = 0; i < 4; i++)
			{
				double percentVbus = mDriveCANTalons[i].getOutputVoltage() / mDriveCANTalons[i].getBusVoltage();
				SmartDashboard.putString ("Wheel #" + i + " SpeedMode | Output | RPS | Setpoint | F :", 
						String.format("%s | %01.2f | %03.2f | %03.2f | %01.3f",
						((mShifterSolenoid.get() == DriveConstants.FAST_SHIFT_DIR) ? "FAST" : "SLOW"),
						percentVbus, 
						mDriveEncoders[i].getRPS(),
						mDriveCANTalons[i].getSetpoint() * DriveConstants.RAW_TO_RPS,
						mDriveCANTalons[i].getF()));
			}
		}
	}
	
	public double getPressure ()
	{
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			return 120;
		}
		else
		{
			double pressureNonAverage = 
					(mPressureSensor.getVoltage() / PressureConstants.PRESSURE_SENSOR_NORMALIZED_VOLTAGE) * 250 - 25;;
			double pressureAveraged = 
					(mPressureSensor.getAverageVoltage() / PressureConstants.PRESSURE_SENSOR_NORMALIZED_VOLTAGE) * 250 - 25;;
					
			return RunConstants.USING_AVERAGE_VOLTAGE_FOR_PRESSURE ? pressureAveraged : pressureNonAverage;
		}
	}
	
	public void performFullCode() {
		@SuppressWarnings("unused")
		long prevTimeDif = -1;
		@SuppressWarnings("unused")
		boolean wasPressed = false;
		
		mDriveTrain.setPOV (RunConstants.DRIVE_DOWN_FIELD_POV);

		while (isOperatorControl() && isEnabled()) {
			SmartDashboard.putString("CURRENT ROBOT MODE:", "TELEOP");
			/*
			if (mPDP.getVoltage() < DriveConstants.EMERGENCY_VOLTAGE)
			{
				mEmergencyMode = true;
				for (int i = 0; i < 4; i++)
				{
					mDriveCANTalons[i].configPeakOutputVoltage(
							-DriveConstants.MAX_VOLTAGE_EMERGENCY, DriveConstants.MAX_VOLTAGE_EMERGENCY);
				}
			}*/
			
			
			SmartDashboard.putBoolean("Is Emergency Mode:", mEmergencyMode);
			
			long timeStart = System.currentTimeMillis();

			//handle subsytems
			SmartDashboard.putNumber("Robot Angle:", mRobotAngle.getAngleDegrees());
			SmartDashboard.putNumber("Current Pressure:", getPressure());
			SmartDashboard.putNumber("Left Sonar:", mLeftSonar.getRangeInches());
			
			//mClimber.update();
			mClimber.updateSimple();
			
			long timeFromStart = System.currentTimeMillis() - mTimeStart;
			
			boolean shouldRunCompressor = false;
			double pressure = getPressure();
			if (timeFromStart < PressureConstants.TIME_RUN_COMPRESSOR)
			{
				mRefillingLateGame = false;
				shouldRunCompressor = true;
			}
			else
			{
				if (mRefillingLateGame)
				{
					if (pressure > PressureConstants.UPPER_PRESSURE_CUTOFF)
					{
						mRefillingLateGame = false;
						shouldRunCompressor = false;
					}
					else
					{
						mRefillingLateGame = true;
						shouldRunCompressor = true;
					}
				}
				else
				{
					if (pressure < PressureConstants.LOWER_PRESSURE_CUTOFF)
					{
						mRefillingLateGame = true;
						shouldRunCompressor = true;
					}
					else
					{
						mRefillingLateGame = false;
						shouldRunCompressor = false;
					}
				}
			}
			
			if (RunConstants.RUN_COMPRESSOR_ALWAYS)
			{
				shouldRunCompressor = true;
			}
			
			
			if (mSecondaryStick.getRawButton(ControlPorts.MANUAL_CLIMB_PORT) || mSecondaryStick.getRawButton(ControlPorts.AUTOMATIC_CLIMB_PORT))
			{
				shouldRunCompressor = false;
			}
			
			SmartDashboard.putBoolean("Refilling Late Game:", mRefillingLateGame);
			SmartDashboard.putBoolean("Should Run Compressor:", shouldRunCompressor);
			
			
			if (RunConstants.RUNNING_PNEUMATICS)
			{
				if (shouldRunCompressor && !mCompressor.enabled())
				{
					mCompressor.start();
				}
				if (!shouldRunCompressor && mCompressor.enabled())
				{
					mCompressor.stop();
				}
			}
			else
			{
				mCompressor.stop();
			}
			
			
//			if (timeDif > DriveConstants.MILLIS_TO_LOW_VOLTAGE && prevTimeDif <= DriveConstants.MILLIS_TO_LOW_VOLTAGE)
//			{
//				if (RunConstants.SPEED_PID)
//				{
//					for (int i = 0; i < 4; i++)
//					{
//						mDriveCANTalons[i].configPeakOutputVoltage(DriveConstants.MAX_VOLTAGE_END, -DriveConstants.MAX_VOLTAGE_END);
//					}
//				}
//			}
			if (mXbox.getAButton())
			{
				mDriveTrain.setPOV(true);
				mDriveTrain.setSlowStick(true);
				if (RunConstants.RUNNING_PNEUMATIC_GRABBER)
				{
					mGearGrabber.setGearUp(true);
				}
			}
			
			if (mXbox.getBButton())
			{
				mDriveTrain.setPOV (RunConstants.DRIVE_DOWN_FIELD_POV);
				mDriveTrain.setSlowStick(false);
			}
			
			//do driving
			long timeBeginMacros = System.currentTimeMillis();
			long timeEndMacros = timeBeginMacros;
			long timeBeginDrive = System.currentTimeMillis();
			long timeEndDrive = timeBeginDrive;
			if (RunConstants.USING_MACROS)
			{
				InGameMacro usingMacro = null;
				for (InGameMacro m : mInGameMacros)
				{
					if (m.isTrue(this))
					{
						usingMacro = m;
						break;
					}
				}

				if (usingMacro != null)
				{
					usingMacro.runFrame();
					timeEndMacros = System.currentTimeMillis();
				}	
				else
				{
					timeBeginDrive = System.currentTimeMillis();
					mDriveTrain.drive();
					timeEndDrive = System.currentTimeMillis();
				}
			}
			else
			{
				timeBeginDrive = System.currentTimeMillis();
				mDriveTrain.drive();
				timeEndDrive = System.currentTimeMillis();
			}
			//long timeEndMacrosBeg
			if (RunConstants.USING_MACROS)
			{
				for (InGameMacro m : mInGameMacros)
				{
					m.handleEnding(this);
				}
			}

//			if (!RunConstants.IS_PROTOTYPE_BOT && RunConstants.RUNNING_PNEUMATIC_GRABBER)
//			{
//				handleLeds (mGearGrabber.getGearState());
//			}
		
			long timeLogBegin = System.currentTimeMillis();
			logEverything();
			long timeLogEnd = System.currentTimeMillis();
			//logWheelSpeeds();

			long timeEnd = System.currentTimeMillis();
			mLoopTimeMillis = timeEnd - timeStart;
			SmartDashboard.putNumber("Loop Time Millis:", timeEnd - timeStart);
			SmartDashboard.putNumber("Loop Time Macros:", timeEndMacros - timeBeginMacros);
			SmartDashboard.putNumber("Loop Time Drive:", timeEndDrive - timeBeginDrive);
			SmartDashboard.putNumber("Loop Time Log:", timeLogEnd - timeLogBegin);
			
			
			//mVelocityEstimator.logToSD();
			
			Timer.delay(0.005);
		}
	}

	public void performSwerveTester()
	{
		while (isOperatorControl() && isEnabled())
		{		
			if (RunConstants.RUNNING_PNEUMATIC_SHIFTER)
			{
				if (mXbox.getBumper(Hand.kRight) && mShifterSolenoid.get() == DriveConstants.SLOW_SHIFT_DIR)
				{
					mShifterSolenoid.set (DriveConstants.FAST_SHIFT_DIR);
					for (int i = 0; i < 4; i++)
					{
						mWheels[i].setSpeedMode(true);
					}
				}
				if (mXbox.getBumper(Hand.kLeft) && mShifterSolenoid.get() == DriveConstants.FAST_SHIFT_DIR)
				{
					mShifterSolenoid.set(DriveConstants.SLOW_SHIFT_DIR);
					for (int i = 0; i < 4; i++)
					{
						mWheels[i].setSpeedMode(false);
					}
				}
			}
			
			boolean setpointEnabled = true;
			double setpoint = 0;
			if (mXbox.getYButton())
			{
				setpoint = 0;
			}
			else if (mXbox.getXButton())
			{
				setpoint = 90;
			}
			else if (mXbox.getAButton())
			{
				setpoint = 180;
			}
			else if (mXbox.getBButton())
			{
				setpoint = 270;
			}
			else if (mDriveTrain.getStickMag(Hand.kLeft) > 0.3)
			{
				setpoint = mDriveTrain.getStickAngle(Hand.kLeft);
			}
			else
			{
				setpointEnabled = false;
			}
			
			double speed = mXbox.getTriggerAxis(Hand.kRight) - mXbox.getTriggerAxis(Hand.kLeft);
			if (speed < 0)
			{
				setpoint += 180;
				setpoint = ResourceFunctions.putAngleInRange(setpoint);
			}
			speed = Math.abs(speed);
			if (Math.abs(speed) < 0.1)
			{
				speed = 0;
			}
			
			for (int i = 0; i < 4; i++)
			{
				if (setpointEnabled)
				{
					mWheels[i].setAngle (setpoint);
					
				}
				mWheels[i].setSpeed (speed);
			}
			
			SmartDashboard.putNumber("Turn Setpoint", setpoint);
			SmartDashboard.putNumber("Module Speed", speed);
			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
			}
			
			logEverything();
			logWheelSpeeds();
		}
	}
	
	public void performTuneRotationPID()
	{
		boolean[] turnEnabled = new boolean[] {true, false, false, false};
		
		while (isOperatorControl() && isEnabled())
		{
			boolean setpointEnabled = true;
			double setpoint = 0;
			
			if (mXbox.getYButton())
			{
				setpoint = 0;
			}
			else if (mXbox.getXButton())
			{
				setpoint = 90;
			}
			else if (mXbox.getAButton())
			{
				setpoint = 180;
			}
			else if (mXbox.getBButton())
			{
				setpoint = 270;
			}
			else if (mDriveTrain.getStickMag(Hand.kLeft) > 0.3)
			{
				setpoint = mDriveTrain.getStickAngle(Hand.kLeft);
			}
			else
			{
				setpointEnabled = false;
			}
			
			for (int i = 0; i < 4; i++)
			{
				if (turnEnabled[i] && setpointEnabled)
				{
					mWheels[i].setAngle (setpoint);
					
				}
				mWheels[i].setSpeed (0);
			}
			
			SmartDashboard.putNumber("Turn Setpoint", setpoint);
			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
			}
			
			logEverything();
		}
	}

	/**
	 * Used to find Talon ports, iterates through turn & drive motors with XboxController
	 */
	public void performMotorTest() {
		boolean yPressedPrev = false;
		boolean aPressedPrev = false;

		int motorIndex = 0;

		while (isOperatorControl() && isEnabled()) {
			SmartDashboard.putString("CURRENT ROBOT MODE:", "TELEOP");
			
			if (yPressedPrev && !mXbox.getYButton()) motorIndex++;
			if (aPressedPrev && !mXbox.getAButton()) motorIndex--;

			yPressedPrev = mXbox.getYButton();
			aPressedPrev = mXbox.getAButton();

			motorIndex += 8; motorIndex %= 8;

			SmartDashboard.putNumber("Port:", motorIndex);
			SmartDashboard.putNumber("Robot Angle:", mRobotAngle.getAngleDegrees());

			for (int i = 0; i < 8; i++) {
				SpeedController t = i < 4 ? mTurnMotors[i] : mDriveMotors[i - 4];
				if (i == motorIndex) {	
					double speed = mXbox.getTriggerAxis(Hand.kRight) - mXbox.getTriggerAxis(Hand.kLeft);
					speed *= 0.8;
					//System.out.println("Speed: " + speed);
					SmartDashboard.putNumber("Speed For Test Talons: ", speed);
					t.set(speed);
				}
				else {
					t.set(0);
				}
			}

			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				if (RunConstants.SPEED_PID)
				{
					SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
				}
			}

			if (mXbox.getXButton())
			{
				mGearAlignerMotor.set(0.5);
			}
			else
			{
				mGearAlignerMotor.set(0.0);
			}

			SmartDashboard.putBoolean("Break Beam:", mBreakBeam.get() == GrabberConstants.GEAR_PRESENT_BREAK_BEAM);
			SmartDashboard.putBoolean("Limit Switch:", mLimitSwitch.get() == GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH);
			SmartDashboard.putBoolean("Climber Break Beam:", mWinchBreakbeam.get());
			
			if (RunConstants.USING_LEFT_SONAR)
			{
				SmartDashboard.putNumber("Left Sonar Distance", mLeftSonar.getRangeInches());
			}
			if (RunConstants.USING_RIGHT_SONAR)
			{
				SmartDashboard.putNumber("Right Sonar Distance", mRightSonar.getRangeInches());
			}
			
			logEverything();
			
			Timer.delay(0.005); // wait for a motor update time
		}
	}

	private void performPneumaticsTest() {
		while (this.isOperatorControl() && this.isEnabled()) {
			if (mXbox.getBumper(Hand.kLeft)) mFramePiston.set(GrabberConstants.FRAME_UP);
			else mFramePiston.set(GrabberConstants.FRAME_DOWN);

			if (mXbox.getBumper(Hand.kRight)) mClamperPiston.set(GrabberConstants.CLAMPER_CLOSED);
			else mClamperPiston.set(GrabberConstants.CLAMPER_OPEN);

			if (mXbox.getAButton()) mSnatcherPiston.set(GrabberConstants.SNATCHER_CLOSED);
			else mSnatcherPiston.set(GrabberConstants.SNATCHER_OPEN);

			Timer.delay(0.005);
		}
	}

	private void performDriveConstantsPercent (double speed, int millis) {
		long timeStarted = -1;
		double angleStarted = mRobotAngle.getAngleDegrees();
		int prevDesiredAngle = -1;

		//for (int i = 0; i < 4; i++) mWheels[i].setNudgeMode(true);

		while (this.isOperatorControl() && this.isEnabled()) {
			int desiredAngle = -1;
			if (mXbox.getYButton()) desiredAngle = 0;
			else if (mXbox.getXButton()) desiredAngle = 90;
			else if (mXbox.getAButton()) desiredAngle = 180;
			else if (mXbox.getBButton()) desiredAngle = 270;

			if (desiredAngle != -1 && desiredAngle != prevDesiredAngle) {
				for (int i = 0; i < 4; i++) {
					mWheels[i].set(desiredAngle, 0);
				}
				Timer.delay(3.0); //let it get to the right angle

				timeStarted = System.currentTimeMillis();
				angleStarted = mRobotAngle.getAngleDegrees();
			}

			if (desiredAngle == -1) {
				for (int i = 0; i < 4; i++) {
					mWheels[i].setSpeed(0);
				}
			}

			else {
				long timeDif = System.currentTimeMillis() - timeStarted;
				if (timeDif < millis) {
					for (int i = 0; i < 4; i++) {
						mWheels[i].set(desiredAngle, speed);
					}
					//double dif = 
					double angleOff = ResourceFunctions.continuousAngleDif(mRobotAngle.getAngleDegrees(), angleStarted);

					SmartDashboard.putNumber("Angle Change:", angleOff);
				}
				else {
					for (int i = 0; i < 4; i++) {
						mWheels[i].set(desiredAngle, 0);
					}
				}
			}

			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
			}

			prevDesiredAngle = desiredAngle;
			Timer.delay(0.005);
		}
	}

	private void performSlowSpeedPIDTest()
	{
		DriveConstants.SwerveSpeeds.Real.SPEED_MULT = 0.1;
		DriveConstants.SwerveSpeeds.Real.ANGULAR_SPEED_MULT = 0.2;
		
		while (this.isOperatorControl() && this.isEnabled())
		{
			mDriveTrain.setSlowStick(true);
			mDriveTrain.drive();

			logWheelSpeeds();
		}
	}
	
	private void performRotateTurnMotors (double pSpeed, int pMillis) {
		long timeStarted = -1;

		//for (int i = 0; i < 4; i++) mWheels[i].setNudgeMode(true);
		int prevDir = 0;
		double[] amountRotated = new double[] {0, 0, 0, 0};
		double[] prevAngle = new double[] {0, 0, 0, 0};

		while (this.isOperatorControl() && this.isEnabled()) {
			int desiredDir = 0;
			if (mXbox.getBumper(Hand.kLeft)) desiredDir = -1;
			else if (mXbox.getBumper(Hand.kRight)) desiredDir = 1;

			if (desiredDir != 0 && desiredDir != prevDir) {
				timeStarted = System.currentTimeMillis();
				for (int i = 0; i < 4; i++) {
					prevAngle[i] = mTurnEncoders[i].getAngleDegrees();
					amountRotated[i] = 0;
				}
			}

			if (desiredDir == 0) {
				for (int i = 0; i < 4; i++) {
					mTurnMotors[i].set(0);
				}
			}
			else {
				long timeDif = System.currentTimeMillis() - timeStarted;
				if (timeDif < pMillis) {
					for (int i = 0; i < 4; i++) {
						mTurnMotors[i].set(pSpeed * desiredDir);
						double wheelAngle = mTurnEncoders[i].getAngleDegrees();
						amountRotated[i] += ResourceFunctions.continuousAngleDif(wheelAngle, prevAngle[i]);
						prevAngle[i] = wheelAngle;
					}

				}
				else {
					for (int i = 0; i < 4; i++) {
						mTurnMotors[i].set(0);
					}
				}

			}


			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
				SmartDashboard.putNumber("Turned Degrees #" + i,  amountRotated[i]);
			}

			prevDir = desiredDir;


			Timer.delay(0.005);
		}
	}

	private void performRotateTimer() {
		long timeStarted = -1;
		int prevDesiredAngle = -1;

		//for (int i = 0; i < 4; i++) mWheels[i].setNudgeMode(true);

		while (this.isOperatorControl() && this.isEnabled()) {
			int desiredAngle = -1;
			if (mXbox.getYButton()) desiredAngle = 0;
			else if (mXbox.getXButton()) desiredAngle = 90;
			else if (mXbox.getAButton()) desiredAngle = 180;
			else if (mXbox.getBButton()) desiredAngle = 270;

			if (desiredAngle != -1 && desiredAngle != prevDesiredAngle) {
				timeStarted = System.currentTimeMillis();
			}

			if (desiredAngle != -1) {

				long timeDif = System.currentTimeMillis() - timeStarted;
				for (int i = 0; i < 4; i++) {
					mWheels[i].set(desiredAngle, 0);
					if (!mWheels[i].isInRangeNudge())
					{
						SmartDashboard.putNumber("Time to rotate #" + i, timeDif);
					}
				}

			}

			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
			}


			prevDesiredAngle = desiredAngle;
			Timer.delay(0.005);
		}
	}

	public void performJetsonTest() {
		while (this.isOperatorControl() && this.isEnabled()) {
			double ul_x, ul_y, ur_x, ur_y, ll_x, ll_y, lr_x, lr_y;
			boolean valid;

			valid = mJetsonTable.getBoolean ("IsValid", false);
			ul_x = mJetsonTable.getNumber("UL_X", -1);
			ul_y = mJetsonTable.getNumber("UL_Y", -1);
			ur_x = mJetsonTable.getNumber("UR_X", -1);
			ur_y = mJetsonTable.getNumber("UR_Y", -1);
			ll_x = mJetsonTable.getNumber("LL_X", -1);
			ll_y = mJetsonTable.getNumber("LL_Y", -1);
			lr_x = mJetsonTable.getNumber("LR_X", -1);
			lr_y = mJetsonTable.getNumber("LR_Y", -1);

			SmartDashboard.putBoolean("is valid", valid);
			SmartDashboard.putNumber("ul x", ul_x);
			SmartDashboard.putNumber("ul y", ul_y);
			SmartDashboard.putNumber("ur x", ur_x);
			SmartDashboard.putNumber("ur y", ur_y);
			SmartDashboard.putNumber("ll x", ll_x);
			SmartDashboard.putNumber("ll y", ll_y);
			SmartDashboard.putNumber("lr x", lr_x);
			SmartDashboard.putNumber("lr y", lr_y);


			Timer.delay(0.005);
		}
	}

	public void performLEDTest() {
		while (this.isOperatorControl() && this.isEnabled()) {
			if (mXbox.getBumper(Hand.kRight)) {
				mLEDStrip.fillColor(LEDConstants.RED);
			}
			else if (mXbox.getBumper(Hand.kLeft)) {
				mLEDStrip.fillColor(LEDConstants.GREEN);
			}
			else if (mXbox.getAButton()) {
				mLEDStrip.fillColor(LEDConstants.CYAN);
			}
			else if (mXbox.getBButton()) {
				mLEDStrip.fillColor(LEDConstants.MAGENTA);
			}
			else if (mXbox.getYButton()) {
				mLEDStrip.fillColor(LEDConstants.YELLOW);
			}
			else {
				mLEDStrip.fillColor(LEDConstants.BLUE);
			}

			mLEDStrip.update();

			Timer.delay(0.005);
		}
	}
	
	public void performGyroTest()
	{
		while (this.isOperatorControl() && this.isEnabled())
		{
			SmartDashboard.putNumber("Gyro Angle:", mRobotAngle.getAngleDegrees());
			Timer.delay(0.005);
		}
	}


	/**
	 * Runs the motors with arcade steering.
	 */
	@Override
	public void operatorControl() {
		startGame();
		
		if (!RunConstants.IS_PROTOTYPE_BOT)
		{
			for (int i = 0; i < 4; i++)
			{
				mDriveCANTalons[i].enableBrakeMode(true);
			}
		}

		switch (RunConstants.RUN_MODE) {
		case TEST_TALONS:
			performMotorTest();
			break;
		case TUNE_TURN_PID:
			performTuneRotationPID();
			break;
		case SWERVE_TESTER:
			performSwerveTester();
			break;
		case FULL:
			performFullCode();
			break;
		case TESTING_SLOW_SPEED_PID:
			performSlowSpeedPIDTest();
			break;
		case WINCH:
			performWinchCode();
			break;
		case TEST_PNEUMATICS:
			performPneumaticsTest();
			break;
		case DRIVE_CONSTANT_PERCENT:
			performDriveConstantsPercent(1.0, 2000);
			break;
		case ROTATE_TURN_MOTORS:
			performRotateTurnMotors(0.5, 5000);
			break;
		case ROTATE_TIMER:
			performRotateTimer ();
			break;
		case JETSON_TEST:
			performJetsonTest();
			break;
		case LED_TEST:
			performLEDTest();
			break;
		case GYRO_TEST:
			performGyroTest();
			break;
		case AUTO_TEST:
			testAutonomous();
			break;
		case KARL_TEST:
			WheelTest();
			break;
		}
	}
	

	@Override
	protected void disabled() {
		SmartDashboard.putString("CURRENT ROBOT MODE:", "INITIALIZING DISABLED, 5s delay");
		
		boolean prevPressed = false;
		long timeDisabledStarted = System.currentTimeMillis();
		boolean ended = false;
		
		while (this.isDisabled())
		{
			long timeElapsed = System.currentTimeMillis() - timeDisabledStarted;
			if (timeElapsed > 5000 && !ended)
			{
				endGame();
				ended = true;
				SmartDashboard.putString("CURRENT ROBOT MODE:", "DISABLED");
				
			}
			boolean pressed = mSecondaryStick.getRawButton(ControlPorts.AUTO_CYCLE_PORT);
			if (pressed && !prevPressed)
			{
				mAutoChoice = 
						AutoMode.values()[(mAutoChoice.ordinal()+1) % AutoMode.values().length];
			}
			prevPressed = pressed;
			SmartDashboard.putString("Auto Chocie:", mAutoChoice.toString());
			Timer.delay(0.005);
		}
	}
	
	public void WheelTest ()
	{
		SmartDashboard.putString ("WheelTestCommand", "StartWheelTest");
		long timeStartedVoltage = System.currentTimeMillis();
		long timeStarted = System.currentTimeMillis();
		double voltage = 0;
		double[] voltages = new double[] {2.0, 4.0, 6.0, 8.0, 10.0};
		long timeEachVoltage = 5000;
		int voltageOn = 0;
		boolean reversed = false;
		while (this.isEnabled() && this.isOperatorControl())
		{
			long timeDifSinceStarted = System.currentTimeMillis() - timeStarted;
			SmartDashboard.putNumber("WheelTestTimestamp", timeDifSinceStarted);
			long timeDif = System.currentTimeMillis() - timeStartedVoltage;
			if (voltageOn >= voltages.length)
			{
				voltage = 0.0;
			}
			else
			{
				voltage = voltages[voltageOn];
				if (timeDif > timeEachVoltage)
				{
					for (int i = 0; i < 4; i++)
					{
						mTurnCANTalons[i].set(0);
					}
					Timer.delay(1.0);
					timeStartedVoltage = System.currentTimeMillis();
					if (reversed)
					{
						voltageOn++;
						reversed = false;
					}
					else
					{
						reversed = true;
					}
				}
			}
			
			if (reversed)
			{
				voltage = -voltage;
			}
			
			StringBuilder logString = new StringBuilder();
			addLogValueLong (logString, timeDifSinceStarted);
			addLogValueDouble (logString, mPDP.getVoltage());
			addLogValueDouble (logString, voltage);
			
			
			if (RunConstants.IS_PROTOTYPE_BOT)
			{
				for (int i = 0; i < 4; i++)
				{
					mTurnMotors[i].set(voltage / 12.0);
					mDriveMotors[i].set(0);
				}
				
				for (int i = 0; i < 12; i++)
				{
					addLogValueDouble(logString, 0.0);
				}
				
				
			}
			else
			{
				for (int i = 0; i < 4; i++)
				{
					mTurnMotors[i].set(voltage / mTurnCANTalons[i].getBusVoltage());
					mDriveMotors[i].set(0);
				}
				for (int i = 0; i < 4; i++)
				{
					addLogValueDouble (logString, mTurnCANTalons[i].getOutputVoltage());
				}
				for (int i = 0; i < 4; i++)
				{
					addLogValueDouble (logString, mTurnCANTalons[i].getOutputCurrent());
				}
				for (int i = 0; i < 4; i++)
				{
					addLogValueDouble (logString, mTurnCANTalons[i].getSpeed());
				}
			}
			String finalLogString = logString.toString();
			finalLogString = finalLogString.substring(0, finalLogString.length() - 1);
			SmartDashboard.putString("WheelTestString", finalLogString);	
		}
		SmartDashboard.putString ("WheelTestCommand", "EndWheelTest");
	}

	public void startGame() 
	{
		if (!mGameHasStarted)
		{
			
			if (RunConstants.RUNNING_PNEUMATIC_SHIFTER)
			{
				mShifterSolenoid.set (DriveConstants.SLOW_SHIFT_DIR);		
			}
			
			SmartDashboard.putString ("DashboardCommand", "StartRecording");
			if (RunConstants.RUNNING_PNEUMATIC_GRABBER)
			{
				mGearGrabber.enable();
				mGearGrabberThread = new Thread (mGearGrabber);
				mGearGrabberThread.start();
			}
			
			if (RunConstants.USING_TX1_VISION)
			{
				mPoseEstimator.enable();
				mPoseEstimatorThread = new Thread (mPoseEstimator);
				mPoseEstimatorThread.start();
			}
			
			if (RunConstants.RUNNING_VELOCITY_ESTIMATOR)
			{
				mVelocityEstimatorThread = new Thread (mVelocityEstimator);
				mVelocityEstimatorThread.start();
			}
			
			if (RunConstants.RUNNING_DISPLACEMENT_ESTIMATOR)
			{
				mDisplacementEstimator.enable();
				mDisplacementEstimatorThread = new Thread (mDisplacementEstimator);
				mDisplacementEstimatorThread.start();
			}
			
			mGameHasStarted = true;
			
			switch (mAutoChoice)
			{
			case NOTHING:
				mAutonomousMacro = MyMacroRoutines.NOTHING_MACRO();
				break;
			case STRAIGHT_FORWARD:
				mAutonomousMacro = MyMacroRoutines.DRIVE_STRAIGHT(mDriveTrain);
				break;
			case STRAIGHT_TURN_LEFT_PEG:
				mAutonomousMacro = MyMacroRoutines.DRIVE_STRAIGHT_TURN_LEFT_PEG(mDriveTrain);
				break;
			case STRAIGHT_TURN_RIGHT_PEG:
				mAutonomousMacro = MyMacroRoutines.DRIVE_STRAIGHT_TURN_RIGHT_PEG(mDriveTrain);
				break;
			case MIDDLE_PLACE_GEAR:
				mAutonomousMacro = MyMacroRoutines.MID_STATION_ULTRASONIC (
						mDriveTrain, 
						mGearGrabber, 
						RunConstants.USING_LEFT_SONAR ? mLeftSonar : mRightSonar);
				break;
			case LEFT_PEG_AUTO:
				mAutonomousMacro = MyMacroRoutines.LEFT_STATION_ULTRASONIC(
						mDriveTrain, 
						mGearGrabber, 
						RunConstants.USING_LEFT_SONAR ? mLeftSonar : mRightSonar);
				break;
			case RIGHT_PEG_AUTO:
				mAutonomousMacro = MyMacroRoutines.RIGHT_STATION_ULTRASONIC(
						mDriveTrain, 
						mGearGrabber, 
						RunConstants.USING_LEFT_SONAR ? mLeftSonar : mRightSonar);
				break;
			}
		}
		
		if (!mTimerHasStarted)
		{
			mTimeStart = System.currentTimeMillis();
			mLastTimestampLogger = -1;
			mTimerHasStarted = true;
		}
	}
	
	public void endGame() {
		if (mGameHasStarted)
		{
			SmartDashboard.putString ("DashboardCommand", "EndRecording");
			if (RunConstants.RUNNING_PNEUMATIC_GRABBER) {
				mGearGrabber.disable();
				mGearGrabberThread.interrupt();
				mGearGrabberThread = null;
			}
			
			if (RunConstants.USING_TX1_VISION)
			{
				mPoseEstimator.disable();
				mPoseEstimatorThread.interrupt();
				mPoseEstimatorThread = null;
				
				if (RunConstants.RUNNING_VELOCITY_ESTIMATOR)
				{
					mVelocityEstimatorThread.interrupt();
					mVelocityEstimatorThread = null;
				}
				
			}
			
			if (RunConstants.RUNNING_DISPLACEMENT_ESTIMATOR)
			{
				mDisplacementEstimator.disable();
				mDisplacementEstimatorThread.interrupt();
				mDisplacementEstimatorThread = null;
			}	
			mGameHasStarted = false;
		}
	}
	
	public void addLogValueDouble (StringBuilder pLogString, double pVal)
	{
		pLogString.append (pVal);
		pLogString.append (",");
	}
	
	public void addLogValueInt (StringBuilder pLogString, int pVal)
	{
		pLogString.append (pVal);
		pLogString.append (",");
	}
	
	public void addLogValueLong (StringBuilder pLogString, long pVal)
	{
		pLogString.append (pVal);
		pLogString.append (",");
	}
	
	public void addLogValueBoolean (StringBuilder pLogString, boolean pVal)
	{
		pLogString.append (pVal ? "1" : "0");
		pLogString.append (",");
	}
	
	public void addLogValueEndDouble (StringBuilder pLogString, double pVal)
	{
		pLogString.append (pVal);
	}
	
	public void addLogValueEndInt (StringBuilder pLogString, int pVal)
	{
		pLogString.append (pVal);
	}
	
	public void addLogValueEndBoolean (StringBuilder pLogString, boolean pVal)
	{
		pLogString.append (pVal ? "1" : "0");
	}
	
	public void logEverythingNY()
	{
		if (mLastTimestampLogger < 0)
		{
			mLastTimestampLogger = System.currentTimeMillis();
		}
		
		long time = System.currentTimeMillis();
		long timeElapsed = time - mTimeStart;
		
		SmartDashboard.putBoolean("Game Has Started:", mGameHasStarted);
		SmartDashboard.putNumber("Time Game Started:", mTimeStart);
		SmartDashboard.putNumber("Time Elapsed:", timeElapsed);
		
		
		StringBuilder logString = new StringBuilder();
		
		//for now it is one frame per line
		addLogValueInt(logString, (int)timeElapsed);
		
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			//current
			for (int i = 0; i < 4; i++) {
				//swerves:
				addLogValueDouble (logString, mPDP.getCurrent(2 * i));
				addLogValueDouble (logString, mPDP.getCurrent(2 * i + 1));
			}
			
			addLogValueDouble (logString, mPDP.getCurrent(0));
			addLogValueDouble (logString, mPDP.getCurrent(9));
		}
		else
		{
			//current
			for (int i = 0; i < 4; i++) {
				addLogValueDouble (logString, mDriveCANTalons[i].getOutputCurrent());
				addLogValueDouble (logString, mTurnCANTalons[i].getOutputCurrent());
			}
			
			addLogValueDouble (logString, ((CANTalon) mGearAlignerMotor).getOutputCurrent());
			addLogValueDouble (logString, ((CANTalon) mWinchMotor).getOutputCurrent());
		}

		addLogValueDouble (logString, mCompressor.getCompressorCurrent());
		addLogValueDouble (logString, mPDP.getTotalCurrent());
		addLogValueDouble (logString, mPDP.getVoltage());

		addLogValueBoolean (logString, mBreakBeam.get() == GrabberConstants.GEAR_PRESENT_BREAK_BEAM);
		addLogValueBoolean (logString, mLimitSwitch.get() == GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH);
		
		addLogValueDouble (logString, mRobotAngle.getAngleDegrees());	
		
		boolean compressorOn = mCompressor.getCompressorCurrent() > 5.0;
		
		if (compressorOn) 
		{
			mCompressorTimeTotal += time - mLastTimestampLogger;
			mCompressorTimeContinuous += time - mLastTimestampLogger;
		}
		else 
		{
			mCompressorTimeContinuous = 0;
		}		
		
		addLogValueBoolean (logString, compressorOn);
		
		addLogValueDouble (logString, getPressure());
		
		if (RunConstants.INITIALIZE_WHEEL_PIDS)
		{
			addLogValueBoolean (logString, mDriveTrain.getPOV());
			
			for (int i = 0; i < 4; i++)
			{
				double angle = mWheels[i].getSetpoint();
//				if (mTurnEncoders[i].getAdd180())
//				{
//					angle += 180;
//				}
				angle = ResourceFunctions.putAngleInRange(angle);
				double percentVbus = 0;
				if (RunConstants.IS_PROTOTYPE_BOT)
				{
					percentVbus = mDriveMotors[i].get();
				}
				else
				{
					percentVbus = mDriveCANTalons[i].getOutputVoltage() / mDriveCANTalons[i].getBusVoltage();
				} 
				
				addLogValueDouble (logString, angle);
				addLogValueDouble (logString, mWheels[i].getRealAngle());
				addLogValueDouble (logString, percentVbus);
			}
			
			addLogValueDouble (logString, mDriveTrain.getDesiredRobotVel().getAngle());
			addLogValueDouble (logString, mDriveTrain.getDesiredRobotVel().getMagnitude());
			addLogValueDouble (logString, mDriveTrain.getDesiredAngularVel());
		}
		else
		{
			addLogValueInt (logString, 0);
			
			for (int i = 0; i < 4; i++)
			{
				addLogValueInt (logString, 0);
				addLogValueInt (logString, 0);
				addLogValueInt (logString, 0);
			}
			
			addLogValueInt (logString, 0);
			addLogValueInt (logString, 0);
			addLogValueInt (logString, 0);
		}
		
		if (RunConstants.RUNNING_PNEUMATIC_SHIFTER)
		{
			addLogValueBoolean (logString, mShifterSolenoid.get() == DriveConstants.FAST_SHIFT_DIR);
		}
		else
		{
			addLogValueBoolean (logString, true);
		}
		
		
		
		if (RunConstants.RUNNING_PNEUMATIC_GRABBER)
		{
			addLogValueInt (logString, mGearGrabber.getGearStateNum());
		}
		else
		{
			addLogValueInt (logString, 0);
		}
		
		addLogValueBoolean (logString, mXbox.getYButton());
		addLogValueBoolean (logString, mXbox.getBButton());
		addLogValueBoolean (logString, mXbox.getAButton());
		addLogValueBoolean (logString, mXbox.getXButton());
		addLogValueBoolean (logString, mXbox.getBumper (Hand.kLeft));
		addLogValueBoolean (logString, mXbox.getBumper (Hand.kRight));
		addLogValueDouble (logString, mXbox.getTriggerAxis (Hand.kLeft));
		addLogValueDouble (logString, mXbox.getTriggerAxis (Hand.kRight));
		addLogValueInt (logString, mXbox.getPOV());
		addLogValueBoolean (logString, mXbox.getStartButton());
		addLogValueBoolean (logString, mXbox.getBackButton());
		addLogValueDouble (logString, mXbox.getX(Hand.kLeft));
		addLogValueDouble (logString, mXbox.getY(Hand.kLeft));
		addLogValueDouble (logString, mXbox.getX(Hand.kRight));
		addLogValueDouble (logString, mXbox.getY(Hand.kRight));
		
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.LEFT_STATION_BUTTON));
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.MIDDLE_STATION_BUTTON));
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.RIGHT_STATION_BUTTON));
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.CYCLE_PLACEMENT_PORT));
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.FLAP_PORT));
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.EMERGENCY_OPEN_PORT));
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.AUTOMATIC_CLIMB_PORT));
		addLogValueBoolean (logString, mSecondaryStick.getRawButton (ControlPorts.MANUAL_CLIMB_PORT));
		addLogValueBoolean (logString, false);
		
		addLogValueEndInt (logString, (int) mLoopTimeMillis);
		
		//logString.append (",");
		
		/*
		logString.append (mCompressorTimeTotal);
		logString.append (",");
		
		logString.append (mCompressorTimeContinuous);
		*/
		
		SmartDashboard.putNumber("TimeLeft", 150000 - timeElapsed);
		SmartDashboard.putString ("LogString", logString.toString());
		
		mLastTimestampLogger = time;
	}
	
	public void logEverythingHVR()
	{

		if (mLastTimestampLogger < 0)
		{
			mLastTimestampLogger = System.currentTimeMillis();
		}
		
		long time = System.currentTimeMillis();
		long timeElapsed = time - mTimeStart;
		
		SmartDashboard.putBoolean("Game Has Started:", mGameHasStarted);
		SmartDashboard.putNumber("Time Game Started:", mTimeStart);
		SmartDashboard.putNumber("Time Elapsed:", timeElapsed);
		
		
		StringBuilder logString = new StringBuilder();
		
		//for now it is one frame per line
		logString.append(timeElapsed);
		logString.append(",");
		
		if (RunConstants.IS_PROTOTYPE_BOT)
		{
			//current
			for (int i = 0; i < 4; i++) {
				//swerves:
				logString.append (mPDP.getCurrent(2 * i));
				logString.append (",");
				
				logString.append (mPDP.getCurrent(2 * i + 1));
				logString.append (",");
								
			}
			
			//squishy:
			logString.append (mPDP.getCurrent(8));
			
			logString.append(",");
			//cilmber:
			logString.append (mPDP.getCurrent(9));
			logString.append(",");
			
			
		}
		else
		{
			//current
			for (int i = 0; i < 4; i++) {
				logString.append (mDriveCANTalons[i].getOutputCurrent());
				logString.append (",");				
				
				logString.append (mTurnCANTalons[i].getOutputCurrent());
				logString.append (",");
			}
			
			logString.append(((CANTalon) mGearAlignerMotor).getOutputCurrent());
			logString.append (",");
			
			logString.append(((CANTalon) mWinchMotor).getOutputCurrent());
			logString.append (",");
						
		}

		logString.append (mCompressor.getCompressorCurrent());
		logString.append (",");

		logString.append (mPDP.getTotalCurrent());
		logString.append (",");

		logString.append (mPDP.getVoltage());
		logString.append (",");

		
		logString.append((mBreakBeam.get() == GrabberConstants.GEAR_PRESENT_BREAK_BEAM) ? "1" : "0");
		logString.append(",");
		
		logString.append((mLimitSwitch.get() == GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH) ? "1" : "0");
		logString.append(",");
		
		logString.append(mWinchBreakbeam.get() ? "1" : "0");
		logString.append(",");		
		
		
		logString.append(mRobotAngle.getAngleDegrees());
		logString.append(",");			
		
		boolean compressorOn = mCompressor.getPressureSwitchValue();
		
		if (compressorOn) {
			
			mCompressorTimeTotal += time - mLastTimestampLogger;
			mCompressorTimeContinuous += time - mLastTimestampLogger;
		}
		else {
			mCompressorTimeContinuous = 0;
		}		
		
		logString.append (compressorOn ? "1" : "0");
		//logString.append (",");
		
		/*
		logString.append (mCompressorTimeTotal);
		logString.append (",");
		
		logString.append (mCompressorTimeContinuous);
		*/
		
		SmartDashboard.putNumber("TimeLeft", 150000 - timeElapsed);
		SmartDashboard.putString ("LogString", logString.toString());
		
		mLastTimestampLogger = time;
	}
	
	public void logEverything()
	{
		if (RunConstants.LOG_USING_NY_VERSION)
		{
			logEverythingNY();
		}
		else
		{
			logEverythingHVR();
		}
	}

	public void logEverythingOld() 
	{
		//StringBuilder logString = new StringBuilder();
		if (!RunConstants.IS_PROTOTYPE_BOT)
		{
			//current
			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber ("Turn" + (i+1) + "_Current", mTurnCANTalons[i].getOutputCurrent());
				SmartDashboard.putNumber ("Drive" + (i+1) + "_Current", mDriveCANTalons[i].getOutputCurrent());
				SmartDashboard.putNumber ("Turn" + (i+1) + "_Voltage", mTurnCANTalons[i].getOutputVoltage());
				SmartDashboard.putNumber ("Drive" + (i+1) + "_Voltage", mDriveCANTalons[i].getOutputVoltage());
			}
		}


		//note: make sure pressureSwitchValue() is actually telling us if it is on

		boolean compressorOn = mCompressor.getPressureSwitchValue();
		long timestamp = System.currentTimeMillis() - mTimeStart;
		
		if (compressorOn) {
			mCompressorTimeTotal += timestamp - mLastTimestampLogger;
			mCompressorTimeContinuous += timestamp - mLastTimestampLogger;
		}
		
		else {
			mCompressorTimeContinuous = 0;
		}

		SmartDashboard.putNumber("TotalCurrent", mPDP.getTotalCurrent());
		SmartDashboard.putNumber("TotalVoltage", mPDP.getVoltage());
		SmartDashboard.putNumber("CompressorCurrent", mCompressor.getCompressorCurrent());
		SmartDashboard.putNumber("CompressorOn", compressorOn ? 1 : 0);
		SmartDashboard.putNumber("CompressorTimeTotal", mCompressorTimeTotal);
		SmartDashboard.putNumber("CompressorTimeContinuous", mCompressorTimeContinuous);
		SmartDashboard.putNumber ("Timestamp", timestamp);
		
		mLastTimestampLogger = timestamp;
	}
}