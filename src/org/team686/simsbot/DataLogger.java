package org.team686.simsbot;


import java.util.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class DataLogger 
{
	List<String> dataNames = new ArrayList<>();
	List<String> dataValues = new ArrayList<>();

	public abstract void log();	
	
	public void putBoolean(String name, boolean value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
		
		SmartDashboard.putBoolean(name, value);		
	}
	
	
	public void putNumber(String name, double value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);

		SmartDashboard.putNumber(name, value);		
	}
	
	public void putNumber(String name, int value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
		
		SmartDashboard.putNumber(name, value);		
	}
	
	public void putString(String name, String value)
	{
		String valueString = String.valueOf(value);
		dataNames.add(name);
		dataValues.add(valueString);
		
		SmartDashboard.putString(name, value);		
	}
	
}
