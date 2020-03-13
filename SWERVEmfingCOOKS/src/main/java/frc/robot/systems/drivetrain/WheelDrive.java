package frc.robot.systems.drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * Adapted from 5675's swerve drive.
 */
public class WheelDrive {
	
private CANSparkMax angleMotor;
private CANSparkMax speedMotor;

private PIDController anglePID;
private AnalogInput azimuthEncoder;

private double setpoint;

public double encoderValue;

	/**
	 @param angleMotor The CAN ID of the azimuth controller
	 @param speedMotor The CAN ID of the speed controller
	 @param analogIn   The Analog ID of the azimuth encoder 
	 @param P 		   The P constant for the PID loop
	 @param I 		   The I constant for the PID loop
	 @param D 		   The D constant for the PID loop
 	*/
	public WheelDrive (int angleMotor, int speedMotor, int analogIn, double P, double I, double D) {
		this.angleMotor = new CANSparkMax(angleMotor, MotorType.kBrushless);
		this.speedMotor = new CANSparkMax(speedMotor, MotorType.kBrushless);

		this.azimuthEncoder = new AnalogInput(analogIn);

		anglePID = new PIDController(P, I, D);

		anglePID.enableContinuousInput(0, 5);
		anglePID.setIntegratorRange(-1, 1);
		
		encoderValue = getAzimuth();
	}


	public void drive (double speed, double angle, boolean deadband) {
		if (deadband) {
			speedMotor.set(0);
			angleMotor.set(0);
		}
		else{
			setpoint = anglePID.calculate(getAzimuth(), angle);
			speedMotor.set(speed);
			angleMotor.set(setpoint);
		}
    }
    
	public double getAzimuth() {
		return azimuthEncoder.getVoltage();
	}

	public void stop() {
		speedMotor.set(0);
		angleMotor.set(0);
	}
}