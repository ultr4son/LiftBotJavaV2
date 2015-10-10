package org.usfirst.frc.team2635.robot;

import actuator.IActuator;

public class TwoCarriageLift extends Lift
{
	IActuator<Double> secondLifter;
	@Override
	public void setPosition(double position)
	{
		// TODO Auto-generated method stub
		super.setPosition(position);
		secondLifter.actuate(position);
	}

}
