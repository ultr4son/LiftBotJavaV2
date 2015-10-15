package org.usfirst.frc.team2635.robot;

import com.kauailabs.navx.frc.AHRS;
import com.lakemonsters2635.sensor.ISensor;

public class NavxHeading implements ISensor<Double>
{
	//TODO: implement angle unwrapping.
	AHRS navx;
	public NavxHeading(AHRS navx)
	{
		this.navx = navx;
	}
	@Override
	public Double sense()
	{
		return navx.getAngle();
	}
	
	
}
