package org.usfirst.frc.team2635.robot;

import com.lakemonsters2635.actuator.interfaces.IActuator;

public class Grabber
{
	IActuator<Boolean> grabber;
	public Grabber(IActuator<Boolean> grabber)
	{
		super();
		this.grabber = grabber;
	}
	public void grab(boolean direction)
	{
		grabber.actuate(direction);
	}
}
