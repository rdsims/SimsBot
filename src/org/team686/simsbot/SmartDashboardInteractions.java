package org.team686.simsbot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.team686.lib.joystick.*;
import org.team686.lib.util.Pose;
import org.team686.simsbot.auto.AutoModeBase;
import org.team686.simsbot.auto.modes.*;



// TODO: learn how to make custom LabView? GUI interface


/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions 
{

    SendableChooser<AutoModeOption> autoModeChooser;
    SendableChooser<AutoStartOption> startPositionChooser;
    SendableChooser<Integer> autoShootChooser;
    
    enum AutoModeOption 
    {
        PLACE_PEG("Place Peg"),
        STAND_STILL("Stand Still"),
        DRIVE_STRAIGHT("Drive Straight"),
        SQUARE_PATTERN("Square Pattern"),
    	VISION_DELAY_CALIB("Vision Delay Calibration");
    	
        public final String name;

        AutoModeOption(String name) {
            this.name = name;
        }
    }

    enum AutoStartOption 
    {
        START_POS_1("Position 1", 1, new Pose(1,1,0)), 
        START_POS_2("Position 2", 2, new Pose(2,2,0)), 
        START_POS_3("Position 3", 3, new Pose(3,3,0));

        public String name;
        public int number;
        public Pose startPose;

        AutoStartOption(String _name, int _number, Pose _startPose)
        {
            name = _name;
            number = _number;
            startPose = _startPose;
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
    	autoModeChooser = new SendableChooser<AutoModeOption>();
    	autoModeChooser.addObject(AutoModeOption.STAND_STILL.toString(),    AutoModeOption.STAND_STILL);
    	autoModeChooser.addDefault( AutoModeOption.PLACE_PEG.toString(),      AutoModeOption.PLACE_PEG);
    	autoModeChooser.addObject( AutoModeOption.DRIVE_STRAIGHT.toString(), AutoModeOption.DRIVE_STRAIGHT);
    	autoModeChooser.addObject( AutoModeOption.SQUARE_PATTERN.toString(), AutoModeOption.SQUARE_PATTERN);
    	autoModeChooser.addObject( AutoModeOption.VISION_DELAY_CALIB.toString(), AutoModeOption.VISION_DELAY_CALIB);
    	SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);
    	
    	startPositionChooser = new SendableChooser<AutoStartOption>();
    	startPositionChooser.addDefault(AutoStartOption.START_POS_1.toString(), AutoStartOption.START_POS_1);
    	startPositionChooser.addObject( AutoStartOption.START_POS_2.toString(), AutoStartOption.START_POS_2);
    	startPositionChooser.addObject( AutoStartOption.START_POS_3.toString(), AutoStartOption.START_POS_3);
    	SmartDashboard.putData("Start Position Chooser", startPositionChooser);
    	
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
    
    
    public AutoModeBase getAutoModeSelection() 
    {
    	AutoModeOption selMode = (AutoModeOption)autoModeChooser.getSelected(); 
    	int selLane = startPositionChooser.getSelected().number;
    	
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
    		
    	case VISION_DELAY_CALIB:
			return new CalibrateVisionDelayMode();
			
		default:
            System.out.println("ERROR: unexpected auto mode: " + selMode);
			return new StandStillMode();
    	}
    }


    public Pose getStartPose() 
    {
    	AutoStartOption selection = (AutoStartOption)startPositionChooser.getSelected();
    	return new Pose(selection.startPose);
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
    
   


