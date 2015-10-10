package org.usfirst.frc.team2635.robot;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.RobotDrive;

public class DrivePIDOutput implements PIDOutput
{
	RobotDrive drive;
	public DrivePIDOutput(RobotDrive drive)
	{
		super();
		this.drive = drive;
	}
	@Override
	public void pidWrite(double output)
	{
		drive.arcadeDrive(0, output);
		
	}

}
