package org.usfirst.frc.team2635.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDSource;

public class NavxHeadingPIDSource implements PIDSource
{
	AHRS navx;
	public NavxHeadingPIDSource(AHRS navx)
	{
		this.navx = navx;
	}
	@Override
	public double pidGet()
	{
		return navx.getAngle();
	}

}
