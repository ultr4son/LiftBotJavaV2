package org.usfirst.frc.team2635.robot;

import com.kauailabs.navx.frc.AHRS;
import com.lakemonsters2635.actuator.modules.DriveArcade;
import com.lakemonsters2635.actuator.modules.ActuatorSimple;
import com.lakemonsters2635.actuator.modules.ActuatorSimpleDoubleSolenoid;
import com.lakemonsters2635.composites.CompositeJoystick;
import com.lakemonsters2635.composites.HDrive;
import com.lakemonsters2635.composites.HDrivePneumatic;
import com.lakemonsters2635.composites.HDrivePneumaticClosedLoop;
import com.lakemonsters2635.sensor.modules.OperatorMath;
import com.lakemonsters2635.sensor.modules.OperatorOneShot;
import com.lakemonsters2635.sensor.modules.SensorPOV;
import com.lakemonsters2635.sensor.modules.SensorRawButton;
import com.lakemonsters2635.sensor.modules.SensorRawJoystickAxis;
import com.lakemonsters2635.sensor.modules.OperatorMath.Operator;

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
	
	
	final int JOYSTICK_CHANNEL = 0;
	
	final String INCREMENT_TOTE_LIFT = "IncrementToteLift";
	final String DECREMENT_TOTE_LIFT = "DecrementToteLift";
	final String OPEN_TOTE_LIFT = "OpenToteLift";
	final String CLOSE_TOTE_LIFT = "CloseToteLift";
	final int INCREMENT_TOTE_LIFT_BUTTON = 4;
	final int DECREMENT_TOTE_LIFT_BUTTON = 1;
	final int OPEN_TOTE_LIFT_BUTTON = 2;
	final int CLOSE_TOTE_LIFT_BUTTON = 3;
	
	final String POV = "pov";
	final int INCREMENT_CAN_LIFT_POV = 0;
	final int DECREMENT_CAN_LIFT_POV = 180;
	final int OPEN_CAN_LIFT_POV = 90;
	final int CLOSE_CAN_LIFT_POV = 270;
	
	final String MOVE = "Y";
	final String ROTATE = "X";
	final String H = "H";
	final int MOVE_AXIS = 1;
	final int ROTATE_AXIS = 0;
	final int H_AXIS = 4;
	
	final int REAR_LEFT_CHANNEL = 2;
	final int REAR_RIGHT_CHANNEL = 5;
	final int FRONT_LEFT_CHANNEL = 3;
	final int FRONT_RIGHT_CHANNEL = 4;
	final int MIDDLE_WHEEL_CHANNEL = 6;
	final int DEPRESSOR_FORWARD = 4;
	final int DEPRESSOR_REVERSE = 5;
	
	final int TOTE_LIFT_MOTOR1_CHANNEL = 8;
	final int TOTE_LIFT_MOTOR2_CHANNEL = 9;
	final int CAN_LIFT_MOTOR_CHANNEL = 7;
	
	final int TOTE_GRABBER_FORWARD_CHANNEL = 0;
	final int TOTE_GRABBER_REVERSE_CHANNEL = 1;
	
	final int CAN_GRABBER_FORWARD_CHANNEL = 2;
	final int CAN_GRABBER_REVERSE_CHANNEL = 3;
	
	final double HDRIVE_P_DEFAULT = 0.0;
	final String HDRIVE_P_KEY = "HDriveP";
	final double HDRIVE_I_DEFAULT = 0.0;
	final String HDRIVE_I_KEY = "HDriveI";
	final double HDRIVE_D_DEFAULT = 0.0;
	final String HDRIVE_D_KEY = "HDriveD";
	
	final double TOTE_P_DEFAULT = 0.1;
	final String TOTE_P_KEY = "ToteP";
	final double TOTE_I_DEFAULT = 0.0;
	final String TOTE_I_KEY = "ToteI";
	final double TOTE_D_DEFAULT = 0.0;
	final String TOTE_D_KEY = "ToteD";

	final double CAN_P_DEFAULT = 0.1;
	final String CAN_P_KEY = "CanP";
	final double CAN_I_DEFAULT = 0.0;
	final String CAN_I_KEY = "CanI";
	final double CAN_D_DEFAULT = 0.0;
	final String CAN_D_KEY = "CanD";
	double DEPRESSION_TOLERANCE = 0.1;
	double scalingFactor = 1.0;
	
	Joystick xboxControllerSource;
	CompositeJoystick xboxController;
	
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
	double[] TOTE_POSITIONS = {0, 1000.0, 2500.0, 5000.0, 7500.0, 10000.0, 12500 };
	int canIndex = 0;
	
	double[] CAN_POSITIONS = {0, 4000.0, 8000.0};

	DoubleSolenoid totePiston;
	DoubleSolenoid canPiston;
	
	Grabber toteArms;
	Grabber canArms;
	
	
	
	Preferences robotPreferences = Preferences.getInstance();
	
	public void robotInit()
	{
		xboxControllerSource = new Joystick(JOYSTICK_CHANNEL);
		xboxController = new CompositeJoystick();
			//Make axes that allow scaling
			xboxController.addAxis(MOVE, new SensorRawJoystickAxis(MOVE_AXIS, xboxControllerSource),
					new OperatorMath(Operator.MULTIPLY));
			xboxController.addAxis(ROTATE, new SensorRawJoystickAxis(ROTATE_AXIS, xboxControllerSource),
					new OperatorMath(Operator.MULTIPLY));
			xboxController.addAxis(H, new SensorRawJoystickAxis(H_AXIS, xboxControllerSource),
					new OperatorMath(Operator.MULTIPLY));
			//Make buttons that are one shots
			xboxController.addButton(CLOSE_TOTE_LIFT, new SensorRawButton(CLOSE_TOTE_LIFT_BUTTON, xboxControllerSource),
					new OperatorOneShot<Boolean, Boolean>(false));
			xboxController.addButton(OPEN_TOTE_LIFT, new SensorRawButton(OPEN_TOTE_LIFT_BUTTON, xboxControllerSource),
					new OperatorOneShot<Boolean, Boolean>(false));
			xboxController.addButton(INCREMENT_TOTE_LIFT, new SensorRawButton(INCREMENT_TOTE_LIFT_BUTTON, xboxControllerSource),
					new OperatorOneShot<Boolean, Boolean>(false));
			xboxController.addButton(DECREMENT_TOTE_LIFT, new SensorRawButton(DECREMENT_TOTE_LIFT_BUTTON, xboxControllerSource),
					new OperatorOneShot<Boolean, Boolean>(false));
			
			xboxController.addPOV(POV, new SensorPOV(0, xboxControllerSource ));
			
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
		
		hDrive = new HDrivePneumatic(new DriveArcade(drive),
				new ActuatorSimple(middleWheel),
				new ActuatorSimpleDoubleSolenoid(depressor), 
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
		
		toteLift = new Lift(new ActuatorTwoMotor(toteLiftMotor1, toteLiftMotor2));
		canLift = new Lift(new ActuatorSimple(canLiftMotor));
		
		totePiston = new DoubleSolenoid(TOTE_GRABBER_FORWARD_CHANNEL, TOTE_GRABBER_REVERSE_CHANNEL);
		canPiston = new DoubleSolenoid(CAN_GRABBER_FORWARD_CHANNEL, CAN_GRABBER_REVERSE_CHANNEL);
		
		toteArms = new Grabber(new ActuatorSimpleDoubleSolenoid(totePiston));
		canArms = new Grabber(new ActuatorSimpleDoubleSolenoid(canPiston));
		
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
			
		double X = xboxController.getAxis(H, scalingFactor);
		double Y = xboxController.getAxis(MOVE, -scalingFactor);
		double rotation = xboxController.getAxis(ROTATE, -scalingFactor);
			
		//Oneshots don't use their modifiers
		boolean toteIncrement = xboxController.getButton(INCREMENT_TOTE_LIFT, false);
		boolean toteDecrement = xboxController.getButton(DECREMENT_TOTE_LIFT, false);
		
		boolean toteOpen = xboxController.getButton(OPEN_TOTE_LIFT, false);
		boolean toteClose = xboxController.getButton(CLOSE_TOTE_LIFT, false);
		
		double canPOV = xboxController.getPOV(POV);

		hDrive.drive(X, Y, rotation);
		
		
		if(toteIncrement && toteIndex < TOTE_POSITIONS.length - 1 && TOTE_POSITIONS[toteIndex] < CAN_POSITIONS[canIndex])
		{
			++toteIndex;
		}
		if(toteDecrement && toteIndex > 0)
		{
			--toteIndex;
		}
		
		if(toteOpen)
		{
			toteArms.grab(true);
		}
		if(toteClose)
		{
			toteArms.grab(false);
		}
		
		if(canPOV == OPEN_CAN_LIFT_POV)
		{
			canArms.grab(true);
		}
		if(canPOV == CLOSE_CAN_LIFT_POV)
		{
			canArms.grab(false);
		}
		
		if(canPOV == INCREMENT_CAN_LIFT_POV && canIndex < CAN_POSITIONS.length -1)
		{
			canIndex++;
		}
		if(canPOV == DECREMENT_CAN_LIFT_POV && canIndex > 0 && TOTE_POSITIONS[toteIndex] < CAN_POSITIONS[canIndex])
		{
			canIndex--;
		}
		
		toteLift.setPosition(TOTE_POSITIONS[toteIndex]);
		canLift.setPosition(CAN_POSITIONS[canIndex]);
		
		SmartDashboard.putNumber("toteindex", toteIndex);
		SmartDashboard.putNumber("canindex", canIndex);
		SmartDashboard.putBoolean("incrementTote", toteIncrement);
		SmartDashboard.putBoolean("decrementTote", toteDecrement);
		SmartDashboard.putNumber("canPovOneShot", canPOV);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic()
	{

	}

}
