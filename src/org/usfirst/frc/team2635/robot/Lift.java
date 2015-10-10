package org.usfirst.frc.team2635.robot;

import actuator.IActuator;

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
