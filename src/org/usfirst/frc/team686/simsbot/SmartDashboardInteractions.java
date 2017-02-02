package org.usfirst.frc.team686.simsbot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.lib.joystick.*;
import org.usfirst.frc.team686.simsbot.SmartDashboardInteractions.AutonOption;
import org.usfirst.frc.team686.simsbot.SmartDashboardInteractions.JoystickOption;
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

    SendableChooser<AutonOption> autoModeChooser;
    SendableChooser<Integer> autoLaneChooser;
    SendableChooser<Integer> autoShootChooser;
    
    enum AutonOption 
    {
        PLACE_PEG("Place Peg"),
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
    
    
    SendableChooser<JoystickOption> joystickModeChooser;
    
    enum JoystickOption 
    {
        ARCADE_DRIVE("Arcade Drive"),
        TRIGGER_DRIVE("Trigger Drive"),				// works for Xbox controller and Xbox steering wheel
        TANK_DRIVE("Tank Drive"),
        CHEESY_ARCADE_DRIVE("Cheesy Arcade Drive"),
        CHEESY_TRIGGER_DRIVE("Cheesy Trigger Drive"),
        CHEESY_2STICK_DRIVE("Cheesy Two-Stick Drive"),
        ADAM_ARCADE_DRIVE("Adam Arcade Drive"),
        POLAR_ARCADE_DRIVE("Polar Arcade Drive");

        public final String name;

        JoystickOption(String name) {
            this.name = name;
        }
    }
    
    
    public void initWithDefaults() 
    {
    	autoModeChooser = new SendableChooser<AutonOption>();
    	autoModeChooser.addObject(AutonOption.STAND_STILL.toString(),    AutonOption.STAND_STILL);
    	autoModeChooser.addDefault( AutonOption.PLACE_PEG.toString(),      AutonOption.PLACE_PEG);
    	autoModeChooser.addObject( AutonOption.DRIVE_STRAIGHT.toString(), AutonOption.DRIVE_STRAIGHT);
    	autoModeChooser.addObject( AutonOption.SQUARE_PATTERN.toString(), AutonOption.SQUARE_PATTERN);
    	SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);
    	
    	autoLaneChooser = new SendableChooser<Integer>();
    	autoLaneChooser.addDefault("Lane 1", 1);
    	autoLaneChooser.addObject( "Lane 2", 2);
    	autoLaneChooser.addObject( "Lane 3", 3);
    	SmartDashboard.putData("Auto Lane Chooser", autoLaneChooser);
    	
    	autoShootChooser = new SendableChooser<Integer>();
    	autoShootChooser.addDefault("Do Not Shoot", 0);
    	autoShootChooser.addObject( "Shoot", 1);
    	SmartDashboard.putData("Shooting Chooser", autoShootChooser);
    	
    	joystickModeChooser = new SendableChooser<JoystickOption>();
    	joystickModeChooser.addDefault(JoystickOption.ARCADE_DRIVE.toString(),        JoystickOption.ARCADE_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.TRIGGER_DRIVE.toString(),        JoystickOption.TRIGGER_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.TANK_DRIVE.toString(), 	      JoystickOption.TANK_DRIVE);
     	joystickModeChooser.addObject(JoystickOption.CHEESY_ARCADE_DRIVE.toString(),  JoystickOption.CHEESY_ARCADE_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.CHEESY_TRIGGER_DRIVE.toString(), JoystickOption.CHEESY_TRIGGER_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.CHEESY_2STICK_DRIVE.toString(),  JoystickOption.CHEESY_2STICK_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.ADAM_ARCADE_DRIVE.toString(),    JoystickOption.ADAM_ARCADE_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.POLAR_ARCADE_DRIVE.toString(),   JoystickOption.POLAR_ARCADE_DRIVE);
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
			
    	case PLACE_PEG:
        	boolean isShooting = ((int)autoShootChooser.getSelected() == 1);
			return new AutoPlacePegMode(selLane, isShooting);
			
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
			
    	case TRIGGER_DRIVE:
			return TriggerDriveJoystick.getInstance();
			
    	case TANK_DRIVE:
    		return TankDriveJoystick.getInstance();
    		
    	case CHEESY_ARCADE_DRIVE:
    		return CheesyArcadeDriveJoystick.getInstance();
    		
    	case CHEESY_TRIGGER_DRIVE:
    		return CheesyTriggerDriveJoystick.getInstance();
    		
    	case CHEESY_2STICK_DRIVE:
    		return CheesyTwoStickDriveJoystick.getInstance();

    	case ADAM_ARCADE_DRIVE:
			return AdamArcadeDriveJoystick.getInstance();

    	case POLAR_ARCADE_DRIVE:
    		return PolarArcadeDriveJoystick.getInstance();
    		    		
    	default:
            System.out.println("ERROR: unexpected joystick selection: " + selMode);
			return ArcadeDriveJoystick.getInstance();
    	}
    }
}
    
   


