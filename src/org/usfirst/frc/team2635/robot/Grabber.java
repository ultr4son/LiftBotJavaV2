package org.usfirst.frc.team2635.robot;

import com.lakemonsters2635.actuator.interfaces.BaseActuator;


public class Grabber
{
	BaseActuator<Boolean> grabber;
	public Grabber(BaseActuator<Boolean> grabber)
	{
		super();
		this.grabber = grabber;
	}
	public void grab(boolean direction)
	{
		grabber.actuate(direction);
	}
}
