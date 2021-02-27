package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IronMechEncoder
{
    private WPI_TalonSRX motor;
    private double distancePerPulse;


    public IronMechEncoder( WPI_TalonSRX motor)
    {
        this.motor=motor;
        distancePerPulse=0;
    }

    public void setDistancePerPulse(double distancePerPulse)
    {
        this.distancePerPulse=distancePerPulse;
    }

    public double getDistance()
    {
        return motor.getSelectedSensorPosition()*distancePerPulse;
    }

    public double getRate(){
        return motor.getSelectedSensorVelocity()*distancePerPulse;
    }

    public void reset(){
        motor.setSelectedSensorPosition(0);
    }

}