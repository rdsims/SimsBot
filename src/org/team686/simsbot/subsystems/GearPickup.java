package org.team686.simsbot.subsystems;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;

import org.team686.simsbot.Constants;
import org.team686.simsbot.command_status.DriveCommand;



public class GearPickup extends Subsystem 
{
	private DoubleSolenoid gearSolenoid;
	private static GearPickup instance = new GearPickup();
	public static GearPickup getInstance() { return instance; }

	Drive drive = Drive.getInstance();					
	
	private final CANTalon intakeMotor;
	
	enum GearPickupState
	{
		DEFAULT, INTAKE, SCORE_START, SCORE;
	}
	GearPickupState state = GearPickupState.DEFAULT;
	
	double scoreStartTime;
	
	public GearPickup()
	{
		gearSolenoid = Constants.getDoubleSolenoidModuleChannel(Constants.kHighGearSolenoidId, Constants.kLowGearSolenoidId);
		intakeMotor = new CANTalon(Constants.kIntakeMotorTalonId);
		intakeMotor.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		state = GearPickupState.DEFAULT;
	}

	private void down() 	{ gearSolenoid.set(DoubleSolenoid.Value.kForward); }
	private void up() 		{ gearSolenoid.set(DoubleSolenoid.Value.kReverse); }
	
	private void intake()		{ intakeMotor.set(-1); }
	private void outtake()		{ intakeMotor.set(+1); }
	private void stopIntake()	{ intakeMotor.set(0); }

	public void stop()
	{
		gearSolenoid.set(DoubleSolenoid.Value.kOff); 
		stopIntake(); 
	}
	
	public void reset()
	{
		up();
		stopIntake();
	}

	
	public void run(boolean _intakeControl, boolean _scoreControl)
	{
		switch(state)
		{
		case DEFAULT:
			up();
			stopIntake();
			
			if (_intakeControl)
			{
				state = GearPickupState.INTAKE;
			}
			else if (_scoreControl)
			{
				state = GearPickupState.SCORE_START;
			}
			
			break;
			
		case INTAKE:
			down();
			intake();
			if (!_intakeControl)
			{
				state = GearPickupState.DEFAULT;
			}
			break;
			
		case SCORE_START:
			scoreStartTime = Timer.getFPGATimestamp();
			state = GearPickupState.SCORE;
			
		case SCORE:
			down();
			outtake();
			drive.setOpenLoop(new DriveCommand(-0.5, -0.5));
			
			double now = Timer.getFPGATimestamp();
			double timePassed = now - scoreStartTime;
			
			if (timePassed>0.5)
			{
				state = GearPickupState.DEFAULT;
			}
			break;
		}
	}
	
  
	@Override
	public void zeroSensors() {
		// TODO Auto-generated method stub
		
	}
	
}
