package org.usfirst.frc.team2635.robot;

import com.lakemonsters2635.actuator.interfaces.BaseActuator;


public class Lift
{
	BaseActuator<Double> lifter;
	public Lift(BaseActuator<Double> lifter)
	{
		super();
		this.lifter = lifter;
	}
	public void setPosition(double position)
	{
		lifter.actuate(position);
	}
}
