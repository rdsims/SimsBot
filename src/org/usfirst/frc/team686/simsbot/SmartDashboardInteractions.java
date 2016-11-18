package org.usfirst.frc.team686.simsbot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team686.simsbot.auto.AutoModeBase;
import org.usfirst.frc.team686.simsbot.auto.AutoModeEndedException;
import org.usfirst.frc.team686.simsbot.auto.modes.*;
import org.usfirst.frc.team686.simsbot.control_loop_demo.BangBangControlMode;
import org.usfirst.frc.team686.simsbot.control_loop_demo.BangBangWithToleranceControlMode;
import org.usfirst.frc.team686.simsbot.control_loop_demo.PControlMode;
import org.usfirst.frc.team686.simsbot.control_loop_demo.PDControlMode;
import org.usfirst.frc.team686.simsbot.control_loop_demo.PIDControlMode;
import org.usfirst.frc.team686.simsbot.control_loop_demo.TalonPIDCalibrationMode;



/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions {
/*
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
    
  */  
	
	
    SendableChooser ctrlLoopDemoChooser;
    
    enum CtrlLoopDemoOption
    {
    	DIST_BANG_BANG("DIST_BANG_BANG"), 
    	DIST_BANG_BANG_HYST("DIST_BANG_BANG_HYSTERESIS"), 
    	DIST_P("DIST_P"), 
    	DIST_PI("DIST_PI"), 
    	DIST_PD("DIST_PD"), 
    	DIST_PID("DIST_PID"), 
    	ANG_BANG_BANG("ANG_BANG_BANG"), 
    	ANG_BANG_BANG_HYST("ANG_BANG_BANG_HYSTERESIS"), 
    	ANG_P("ANG_P"), 
    	ANG_PI("ANG_PI"), 
    	ANG_PD("ANG_PD"), 
    	ANG_PID("ANG_PID"),
    	TALON_PID_CAL("TALON_PID_CAL"),
    	DRIVE_FORWARD("DRIVE_FORWARD"), 
    	SIMPLE_PATH("SIMPLE_PATH"), 
    	SQUARE_PATTERN("SQUARE_PATTERN"); 
    	
        public final String name;

    	CtrlLoopDemoOption(String name) {
            this.name = name;
        }
     	
    }
    
    public void initWithDefaults() 
    {
    	ctrlLoopDemoChooser = new SendableChooser();
    	ctrlLoopDemoChooser.addDefault(CtrlLoopDemoOption.DIST_BANG_BANG.toString(),      CtrlLoopDemoOption.DIST_BANG_BANG);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.DIST_BANG_BANG_HYST.toString(), CtrlLoopDemoOption.DIST_BANG_BANG_HYST);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.DIST_P.toString(), CtrlLoopDemoOption.DIST_P);
    	//ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.DIST_PI.toString(), CtrlLoopDemoOption.DIST_PI);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.DIST_PD.toString(), CtrlLoopDemoOption.DIST_PD);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.DIST_PID.toString(), CtrlLoopDemoOption.DIST_PID);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.ANG_BANG_BANG.toString(),      CtrlLoopDemoOption.ANG_BANG_BANG);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.ANG_BANG_BANG_HYST.toString(), CtrlLoopDemoOption.ANG_BANG_BANG_HYST);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.ANG_P.toString(), CtrlLoopDemoOption.ANG_P);
    	//ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.ANG_PI.toString(), CtrlLoopDemoOption.ANG_PI);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.ANG_PD.toString(), CtrlLoopDemoOption.ANG_PD);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.ANG_PID.toString(), CtrlLoopDemoOption.ANG_PID);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.TALON_PID_CAL.toString(), CtrlLoopDemoOption.TALON_PID_CAL);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.DRIVE_FORWARD.toString(), CtrlLoopDemoOption.DRIVE_FORWARD);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.SIMPLE_PATH.toString(), CtrlLoopDemoOption.SIMPLE_PATH);
    	ctrlLoopDemoChooser.addObject( CtrlLoopDemoOption.SQUARE_PATTERN.toString(), CtrlLoopDemoOption.SQUARE_PATTERN);
    	SmartDashboard.putData("Ctrl Loop Demo Chooser", ctrlLoopDemoChooser);
 	
    	SmartDashboard.putNumber("Kp", 1.0);
    	SmartDashboard.putNumber("Kd", 0.0);
    	SmartDashboard.putNumber("Ki", 0.0);
     	
    }

    public AutoModeBase getSelectedAutonMode() 
    {
    	CtrlLoopDemoOption mode = (CtrlLoopDemoOption)ctrlLoopDemoChooser.getSelected(); 
    	double kp = SmartDashboard.getNumber("Kp");
    	double kd = SmartDashboard.getNumber("Kd");
    	double ki = SmartDashboard.getNumber("Ki");
    	
    	
    	switch (mode)
    	{
    	case DIST_BANG_BANG:
			return new BangBangControlMode(0);
			
    	case DIST_BANG_BANG_HYST:
			return new BangBangWithToleranceControlMode(0);
			
    	case DIST_P:
			return new PControlMode(0,kp,kd,ki);
			
//    	case DIST_PI:
//			return new PIControlMode(0,kp,kd,ki);
			
    	case DIST_PD:
			return new PDControlMode(0,kp,kd,ki);
			
    	case DIST_PID:
			return new PIDControlMode(0,kp,kd,ki);
			
    	case ANG_BANG_BANG:
			return new BangBangControlMode(1);
			
    	case ANG_BANG_BANG_HYST:
			return new BangBangWithToleranceControlMode(1);
			
    	case ANG_P:
			return new PControlMode(1,kp,kd,ki);
			
//    	case ANG_PI:
//			return new PIControlMode(1,kp,kd,ki);
			
    	case ANG_PD:
			return new PDControlMode(1,kp,kd,ki);
			
    	case ANG_PID:
			return new PIDControlMode(1,kp,kd,ki);
				
    	case TALON_PID_CAL:
			return new TalonPIDCalibrationMode();
				
    	case DRIVE_FORWARD:
			return new DriveStraightMode(1,false);
				
    	case SIMPLE_PATH:
			return new SimplePathMode(1,false);
				
    	case SQUARE_PATTERN:
			return new SquarePatternMode(1,false);
				
		default:
            System.out.println("ERROR: unexpected auto mode: " + mode);
			return new StandStillMode();
    	}
    }
}
