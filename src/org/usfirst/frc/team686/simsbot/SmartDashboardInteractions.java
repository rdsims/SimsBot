package org.usfirst.frc.team686.simsbot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.modes.*;
//import org.usfirst.frc.team686.util.Rotation2d;


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
        SQUARE_PATTERN("Square Pattern"), //
        STAND_STILL("Stand Still");

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
    
    public void initWithDefaults() 
    {
    	autoModeChooser = new SendableChooser();
    	autoModeChooser.addDefault(AutonOption.STAND_STILL.toString(),    AutonOption.STAND_STILL);
    	autoModeChooser.addObject( AutonOption.SQUARE_PATTERN.toString(), AutonOption.SQUARE_PATTERN);
    	SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);
    	
    	autoLaneChooser = new SendableChooser();
    	autoLaneChooser.addObject( "Lane 1", 1);
    	autoLaneChooser.addObject( "Lane 2", 2);
    	autoLaneChooser.addObject( "Lane 3", 3);
    	autoLaneChooser.addDefault("Lane 4", 4);
    	autoLaneChooser.addObject( "Lane 5", 5);
    	SmartDashboard.putData("Auto Lane Chooser", autoLaneChooser);
    }

    public AutoModeBase getSelectedAutonMode() 
    {
    	AutonOption selMode = (AutonOption)autoModeChooser.getSelected(); 
    	int selLane = (int)autoLaneChooser.getSelected();

    	switch (selMode)
    	{
    	case STAND_STILL:
			return new StandStillMode();
			
    	case SQUARE_PATTERN:
    		return new SquarePatternMode(selLane, false);
    		
		default:
            System.out.println("ERROR: unexpected auto mode: " + selMode);
			return new StandStillMode();
    	}
    }
}
