package frc.robot.inputs.interfaces;

import frc.robot.inputs.models.enums.RestMode;

//import frc.robot.models.enums.*;

public interface IMotor
{
    public void SetIdleMode(RestMode restMode);

    public void SetInverted(boolean isInverted);

    public void Reset();

    public void SetPower(double power);
}