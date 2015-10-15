package org.usfirst.frc.team2635.robot;

import com.kauailabs.navx.frc.AHRS;
import com.lakemonsters2635.actuator.modules.ArcadeDrive;
import com.lakemonsters2635.actuator.modules.SimpleActuator;
import com.lakemonsters2635.actuator.modules.SimpleDoubleSolenoid;
import com.lakemonsters2635.actuator.interfaces.IActuator;
import com.lakemonsters2635.composites.HDrive;
import com.lakemonsters2635.composites.HDrivePneumatic;
import com.lakemonsters2635.composites.HDrivePneumaticClosedLoop;
import com.lakemonsters2635.util.OneShot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.CANTalon.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot
{
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	int JOYSTICK_CHANNEL = 0;
	int INCREMENT_TOTE_LIFT = 4;
	int DECREMENT_TOTE_LIFT = 1;
	int OPEN_TOTE_LIFT = 2;
	int CLOSE_TOTE_LIFT = 3;
	
	int INCREMENT_CAN_LIFT = 0;
	int DECREMENT_CAN_LIFT = 180;
	int OPEN_CAN_LIFT = 90;
	int CLOSE_CAN_LIFT = 270;
	
	int MOVE_AXIS = 1;
	int ROTATE_AXIS = 0;
	int H_AXIS = 4;
	
	int REAR_LEFT_CHANNEL = 2;
	int REAR_RIGHT_CHANNEL = 5;
	int FRONT_LEFT_CHANNEL = 3;
	int FRONT_RIGHT_CHANNEL = 4;
	int MIDDLE_WHEEL_CHANNEL = 6;
	int DEPRESSOR_FORWARD = 4;
	int DEPRESSOR_REVERSE = 5;
	
	int TOTE_LIFT_MOTOR1_CHANNEL = 8;
	int TOTE_LIFT_MOTOR2_CHANNEL = 9;
	int CAN_LIFT_MOTOR_CHANNEL = 7;
	
	int TOTE_GRABBER_FORWARD_CHANNEL = 0;
	int TOTE_GRABBER_REVERSE_CHANNEL = 1;
	
	int CAN_GRABBER_FORWARD_CHANNEL = 2;
	int CAN_GRABBER_REVERSE_CHANNEL = 3;
	
	double HDRIVE_P_DEFAULT = 0.0;
	String HDRIVE_P_KEY = "HDriveP";
	double HDRIVE_I_DEFAULT = 0.0;
	String HDRIVE_I_KEY = "HDriveI";
	double HDRIVE_D_DEFAULT = 0.0;
	String HDRIVE_D_KEY = "HDriveD";
	
	double TOTE_P_DEFAULT = 0.0;
	String TOTE_P_KEY = "ToteP";
	double TOTE_I_DEFAULT = 0.0;
	String TOTE_I_KEY = "ToteI";
	double TOTE_D_DEFAULT = 0.0;
	String TOTE_D_KEY = "ToteD";

	double CAN_P_DEFAULT = 0.0;
	String CAN_P_KEY = "CanP";
	double CAN_I_DEFAULT = 0.0;
	String CAN_I_KEY = "CanI";
	double CAN_D_DEFAULT = 0.0;
	String CAN_D_KEY = "CanD";

	double DEPRESSION_TOLERANCE = 0.1;
	
	CANTalon rearLeftMotor;
	CANTalon rearRightMotor;
	CANTalon frontRightMotor;
	CANTalon frontLeftMotor;
	CANTalon middleWheel;
	PIDController hDrivePid;
	DoubleSolenoid depressor;
	RobotDrive drive;
	AHRS gyroscope;
	HDrivePneumatic hDrive;
	
	CANTalon toteLiftMotor1;
	CANTalon toteLiftMotor2;
	CANTalon canLiftMotor;
	
	Lift toteLift;
	Lift canLift;
	
	//TODO: get the actual values for these from the labview version.
	int toteIndex = 0;
	double[] TOTE_POSITIONS = {0, 1000.0, 2000.0, 4000.0, 8000.0 };
	int canIndex = 0;
	
	double[] CAN_POSITIONS = {0, 1000.0, 2000.0, 4000.0, 8000.0};

	DoubleSolenoid totePiston;
	DoubleSolenoid canPiston;
	
	Grabber toteArms;
	Grabber canArms;
	
	Joystick xboxController;
	OneShot<Boolean> toteIncrementOneShot;
	OneShot<Boolean> toteDecrementOneShot;
	
	OneShot<Double> canIncrementOneShot;
	Preferences robotPreferences = Preferences.getInstance();
	
	public void robotInit()
	{
		xboxController = new Joystick(JOYSTICK_CHANNEL);
		toteIncrementOneShot = new OneShot<Boolean>(false);
		toteDecrementOneShot = new OneShot<Boolean>(false);
		//DRIVE INITIALIZATION
		rearLeftMotor = new CANTalon(REAR_LEFT_CHANNEL);
		rearLeftMotor.changeControlMode(ControlMode.PercentVbus);
		rearRightMotor = new CANTalon(REAR_RIGHT_CHANNEL);
		rearRightMotor.changeControlMode(ControlMode.PercentVbus);
		frontLeftMotor = new CANTalon(FRONT_LEFT_CHANNEL);
		frontLeftMotor.changeControlMode(ControlMode.PercentVbus);
		frontRightMotor = new CANTalon(FRONT_RIGHT_CHANNEL);
		frontRightMotor.changeControlMode(ControlMode.PercentVbus);
		middleWheel = new CANTalon(MIDDLE_WHEEL_CHANNEL);
		depressor = new DoubleSolenoid(DEPRESSOR_FORWARD, DEPRESSOR_REVERSE);
				
		drive = new RobotDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
		gyroscope = new AHRS(SerialPort.Port.kMXP);
		hDrivePid = new PIDController(
				robotPreferences.getDouble(HDRIVE_P_KEY, HDRIVE_P_DEFAULT),
				robotPreferences.getDouble(HDRIVE_I_KEY, HDRIVE_I_DEFAULT),
				robotPreferences.getDouble(HDRIVE_D_KEY, HDRIVE_D_DEFAULT), 
				new NavxHeadingPIDSource(gyroscope),
				new DrivePIDOutput(drive));
		
		hDrive = new HDrivePneumatic(new ArcadeDrive(drive),
				new SimpleActuator(middleWheel),
				new SimpleDoubleSolenoid(depressor), 
				DEPRESSION_TOLERANCE
			);
		//END
		
		//START LIFT INITIALIZATION
		toteLiftMotor1 = new CANTalon(TOTE_LIFT_MOTOR1_CHANNEL);
		toteLiftMotor1.changeControlMode(ControlMode.Position);
		toteLiftMotor1.setPosition(0);
		toteLiftMotor1.reverseSensor(true);
		toteLiftMotor1.setPID(
				robotPreferences.getDouble(TOTE_P_KEY, TOTE_P_DEFAULT),
				robotPreferences.getDouble(TOTE_I_KEY, TOTE_I_DEFAULT),
				robotPreferences.getDouble(TOTE_D_KEY, TOTE_D_DEFAULT));
			
		toteLiftMotor2 = new CANTalon(TOTE_LIFT_MOTOR2_CHANNEL);
		toteLiftMotor2.changeControlMode(ControlMode.Follower);
		toteLiftMotor2.setPosition(0);
		
		canLiftMotor = new CANTalon(CAN_LIFT_MOTOR_CHANNEL);
		canLiftMotor.changeControlMode(ControlMode.Position);
		canLiftMotor.reverseOutput(true);
		canLiftMotor.setPosition(0);
		canLiftMotor.setPID(
				robotPreferences.getDouble(CAN_P_KEY, CAN_P_DEFAULT),
				robotPreferences.getDouble(CAN_I_KEY, CAN_I_DEFAULT),
				robotPreferences.getDouble(CAN_D_KEY, CAN_D_DEFAULT));
		
		toteLift = new Lift(new TwoMotorActuator(toteLiftMotor1, toteLiftMotor2));
		canLift = new Lift(new SimpleActuator(canLiftMotor));
		
		totePiston = new DoubleSolenoid(TOTE_GRABBER_FORWARD_CHANNEL, TOTE_GRABBER_REVERSE_CHANNEL);
		canPiston = new DoubleSolenoid(CAN_GRABBER_FORWARD_CHANNEL, CAN_GRABBER_REVERSE_CHANNEL);
		
		toteArms = new Grabber(new SimpleDoubleSolenoid(totePiston));
		canArms = new Grabber(new SimpleDoubleSolenoid(canPiston));
		
		//END
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic()
	{
		
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopInit()
	{
		toteLiftMotor1.setPID(
				robotPreferences.getDouble(TOTE_P_KEY, TOTE_P_DEFAULT),
				robotPreferences.getDouble(TOTE_I_KEY, TOTE_I_DEFAULT),
				robotPreferences.getDouble(TOTE_D_KEY, TOTE_D_DEFAULT));
		canLiftMotor.setPID(
				robotPreferences.getDouble(CAN_P_KEY, CAN_P_DEFAULT),
				robotPreferences.getDouble(CAN_I_KEY, CAN_I_DEFAULT),
				robotPreferences.getDouble(CAN_D_KEY, CAN_D_DEFAULT));

		
	}
	public void teleopPeriodic()
	{
			
			//MORE LIKE TRIPLE X AMIRITE
			double X = xboxController.getRawAxis(H_AXIS);
			double Y = -xboxController.getRawAxis(MOVE_AXIS);
			double rotation = -xboxController.getRawAxis(ROTATE_AXIS);
				
			boolean toteIncrement = toteIncrementOneShot.oneShot(xboxController.getRawButton(INCREMENT_TOTE_LIFT));
			boolean toteDecrement = toteDecrementOneShot.oneShot(xboxController.getRawButton(DECREMENT_TOTE_LIFT));
			
			boolean toteOpen = xboxController.getRawButton(OPEN_TOTE_LIFT);
			boolean toteClose = xboxController.getRawButton(CLOSE_TOTE_LIFT);
			
			double canPOV = xboxController.getPOV();
	
		hDrive.drive(X, Y, rotation);
		
		
		if(toteIncrement && toteIndex < TOTE_POSITIONS.length - 1 && toteIndex < canIndex)
		{
			++toteIndex;
		}
		if(toteDecrement && toteIndex > 0)
		{
			--toteIndex;
		}
		
		//if toteClose is true, it will be set false and the tote arms will close.
		if(toteOpen)
		{
			toteArms.grab(true);
		}
		if(toteClose)
		{
			toteArms.grab(false);
		}
		
		if(canPOV == OPEN_CAN_LIFT)
		{
			canArms.grab(true);
		}
		if(canPOV == CLOSE_CAN_LIFT)
		{
			canArms.grab(false);
		}
		
		if(canPOV == INCREMENT_CAN_LIFT && canIndex < CAN_POSITIONS.length -1)
		{
			canIndex++;
		}
		if(canPOV == DECREMENT_CAN_LIFT && canIndex > 0 && canIndex > toteIndex)
		{
			canIndex--;
		}
		
		toteLift.setPosition(TOTE_POSITIONS[toteIndex]);
		canLift.setPosition(CAN_POSITIONS[canIndex]);
		
		SmartDashboard.putNumber("toteindex", toteIndex);
		SmartDashboard.putNumber("canindex", canIndex);
		SmartDashboard.putBoolean("incrementTote", toteIncrement);
		SmartDashboard.putBoolean("decrementTote", toteDecrement);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic()
	{

	}

}
