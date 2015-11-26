package org.usfirst.frc.team2635.robot;

import edu.wpi.first.wpilibj.SpeedController;

import com.lakemonsters2635.actuator.interfaces.BaseActuator;
public class ActuatorTwoMotor extends BaseActuator<Double>
{
	SpeedController motor1;
	SpeedController motor2;
	public ActuatorTwoMotor(SpeedController motor1, SpeedController motor2)
	{
		super();
		this.motor1 = motor1;
		this.motor2 = motor2;
	}
	@Override
	public boolean actuate(Double magnitude)
	{
		motor1.set(magnitude);
		motor2.set(magnitude);
		return true;
	}
	
	
}
