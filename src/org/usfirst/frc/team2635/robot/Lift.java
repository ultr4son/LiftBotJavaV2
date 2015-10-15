package org.usfirst.frc.team2635.robot;

import com.lakemonsters2635.actuator.interfaces.IActuator;

public class Lift
{
	IActuator<Double> lifter;
	public Lift(IActuator<Double> lifter)
	{
		super();
		this.lifter = lifter;
	}
	public void setPosition(double position)
	{
		lifter.actuate(position);
	}
}
