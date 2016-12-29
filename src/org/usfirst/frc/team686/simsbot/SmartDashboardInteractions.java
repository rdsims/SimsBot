package org.usfirst.frc.team686.simsbot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.joystick.*;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.modes.*;




/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions {

    SendableChooser autoModeChooser;
    SendableChooser autoLaneChooser;
    
    enum AutonOption 
    {
        STAND_STILL("Stand Still"),
        DRIVE_STRAIGHT("Drive Straight"),
        SQUARE_PATTERN("Square Pattern");

        public final String name;

        AutonOption(String name) {
            this.name = name;
        }
    }

    enum AutonLane 
    {
        LANE_1(160, "1"), LANE_2(205, "2"), LANE_3(160, "3"), LANE_4(155, "4"), LANE_5(220, "5");

        public final double distanceToDrive;
        public final String numberString;

        AutonLane(double distanceToDrive, String numberString)
        {
            this.distanceToDrive = distanceToDrive;
            this.numberString = numberString;
        }
    }
    
    
    SendableChooser joystickModeChooser;
    
    enum JoystickOption 
    {
        ARCADE_DRIVE("Arcade Drive"),
        ADAM_ARCADE_DRIVE("Adam Arcade Drive"),
        TRIGGER_DRIVE("Trigger Drive"),
        TANK_DRIVE("Tank Drive");

        public final String name;

        JoystickOption(String name) {
            this.name = name;
        }
    }
    
    
    public void initWithDefaults() 
    {
    	autoModeChooser = new SendableChooser();
    	autoModeChooser.addDefault(AutonOption.STAND_STILL.toString(),    AutonOption.STAND_STILL);
    	autoModeChooser.addObject( AutonOption.DRIVE_STRAIGHT.toString(), AutonOption.DRIVE_STRAIGHT);
    	autoModeChooser.addObject( AutonOption.SQUARE_PATTERN.toString(), AutonOption.SQUARE_PATTERN);
    	SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);
    	
    	autoLaneChooser = new SendableChooser();
    	autoLaneChooser.addObject( "Lane 1", 1);
    	autoLaneChooser.addObject( "Lane 2", 2);
    	autoLaneChooser.addObject( "Lane 3", 3);
    	autoLaneChooser.addDefault("Lane 4", 4);
    	autoLaneChooser.addObject( "Lane 5", 5);
    	SmartDashboard.putData("Auto Lane Chooser", autoLaneChooser);
    	
    	joystickModeChooser = new SendableChooser();
    	joystickModeChooser.addDefault(JoystickOption.ARCADE_DRIVE.toString(),        JoystickOption.ARCADE_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.ADAM_ARCADE_DRIVE.toString(),    JoystickOption.ADAM_ARCADE_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.TRIGGER_DRIVE.toString(),        JoystickOption.TRIGGER_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.TANK_DRIVE.toString(), 	      JoystickOption.TANK_DRIVE);
    	SmartDashboard.putData("Joystick Chooser", joystickModeChooser);
    	
     }
    
    public AutoModeBase getSelectedAutonMode() 
    {
    	AutonOption selMode = (AutonOption)autoModeChooser.getSelected(); 
    	int selLane = (int)autoLaneChooser.getSelected();
    	
    	switch (selMode)
    	{
    	case STAND_STILL:
			return new StandStillMode();
			
    	case DRIVE_STRAIGHT:
			return new DriveStraightMode(selLane, false);
			
    	case SQUARE_PATTERN:
    		return new SquarePatternMode(selLane, false);
    		
		default:
            System.out.println("ERROR: unexpected auto mode: " + selMode);
			return new StandStillMode();
    	}
    }


    public JoystickControlsBase getJoystickControlsMode() 
    {
    	JoystickOption selMode = (JoystickOption)joystickModeChooser.getSelected(); 
    	
    	switch (selMode)
    	{
    	case ARCADE_DRIVE:
			return ArcadeDriveJoystick.getInstance();
			
    	case ADAM_ARCADE_DRIVE:
			return AdamArcadeDriveJoystick.getInstance();

    	case TRIGGER_DRIVE:
			return TriggerDriveJoystick.getInstance();
			
    	case TANK_DRIVE:
    		return TankDriveJoystick.getInstance();
    		
		default:
            System.out.println("ERROR: unexpected joystick selection: " + selMode);
			return ArcadeDriveJoystick.getInstance();
    	}
    }
}
    
   


