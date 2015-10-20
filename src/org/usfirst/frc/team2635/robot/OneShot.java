package org.usfirst.frc.team2635.robot;

public class OneShot<T> 
{
	T previousValue;
	T lowValue;
	public OneShot(T lowValue)
	{
		this.lowValue = lowValue;
		this.previousValue = lowValue;
	}
	public T oneShot(T currentValue)
	{
		//If T is Integer, Double, etc., .equals will compare by value, not by object
		if(!currentValue.equals(previousValue))
		{
			System.out.println("Hit: " + currentValue + "Previous Value: " + previousValue);
			previousValue = currentValue;
			return currentValue;
		}
		return lowValue;
	}
}
