package frc.robot.inputs.interfaces;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.inputs.models.enums.RestMode;

public class Talon implements IMotor 
{
    private WPI_TalonSRX _talon;

    public Talon(int canID) {
        this(canID, true);
    }

    public Talon(int canID, boolean reset) {
        _talon = new WPI_TalonSRX(canID);
        if (reset) 
            _talon.configFactoryDefault();
    }

    public void SetIdleMode(RestMode restMode) {
        if (restMode == RestMode.COAST)
            _talon.setNeutralMode(NeutralMode.Coast);
        if (restMode == RestMode.BRAKE)
            _talon.setNeutralMode(NeutralMode.Brake);
    }
    
    public void SetInverted(boolean setInverted) {
        _talon.setInverted(setInverted);
    }

    public void Reset() {
        _talon.configFactoryDefault();
    }

    /** Sets power in percen mode valid values between -1.0 and 1.0 */
    public void SetPower(double power) {
        _talon.set(ControlMode.PercentOutput, power);
    }

    public SpeedController GetController()
    {
        return _talon;
    }
    
    public WPI_TalonSRX GetWPITalon()
    {
        return _talon;
    }
}