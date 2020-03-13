package frc.robot.inputs.interfaces;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedController;
import frc.robot.inputs.models.enums.RestMode;

public class SparkMax implements IMotor
{
    private CANSparkMax _sparkMax;
    
    public SparkMax(int canID)
    {
        this(canID, MotorType.kBrushless);
    }

    public SparkMax(int canID, MotorType motorType)
    {
        this(canID, motorType, RestMode.COAST);
    }
    
    public SparkMax(int canID, MotorType motorType, RestMode restMode)
    {
        this(canID, motorType, restMode, false);
    }

    public SparkMax(int canId, MotorType motorType, RestMode restMode, boolean isInverted)
    {
        _sparkMax = new CANSparkMax(canId, MotorType.kBrushless);
        _sparkMax.restoreFactoryDefaults();

        if(motorType == MotorType.kBrushed)
            _sparkMax = new CANSparkMax(canId, MotorType.kBrushed);
        if(motorType == MotorType.kBrushless)
            _sparkMax = new CANSparkMax(canId, MotorType.kBrushless);
        
        if(restMode == RestMode.COAST)
            _sparkMax.setIdleMode(IdleMode.kCoast);
        if(restMode == RestMode.BRAKE)  
            _sparkMax.setIdleMode(IdleMode.kBrake);

        _sparkMax.setInverted(isInverted);
    }

    public void SetIdleMode(RestMode restMode)
    {
        if(restMode == RestMode.COAST)
            _sparkMax.setIdleMode(IdleMode.kCoast);
        if(restMode == RestMode.BRAKE)  
            _sparkMax.setIdleMode(IdleMode.kBrake);
    }

    public void SetInverted(boolean setInverted)
    {
        _sparkMax.setInverted(setInverted);
    }

    public void Reset()
    {
        _sparkMax.restoreFactoryDefaults();
    }

    public void SetPower(double power)
    {
        _sparkMax.set(power);
    }

    public SpeedController GetController()
    {
        return _sparkMax;
    }
    
    public CANSparkMax GetSparkMax()
    {
        return _sparkMax;
    }
}