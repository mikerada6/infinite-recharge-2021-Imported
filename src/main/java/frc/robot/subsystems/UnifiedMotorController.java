/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*a sort of master class for the function of running motors in two directions, the ID of the motor is specified when calling the
*methods in the command.*/

public class UnifiedMotorController extends SubsystemBase {
    protected double motorPercentage;
    protected Victor motor;

    public UnifiedMotorController()
    {
    }

    public void setConstant(int c){
        this.motor = new Victor(c);
    }
    //runs the motor in a "forwards" direction
    public void runForward(double _motorPercentage) {
        this.motorPercentage=_motorPercentage;
        motor.set(motorPercentage);
        
    }
    //runs the motor in a "backwards" direction
    public void runBackwards(double _motorPercentage) {
        this.motorPercentage=_motorPercentage * -1;
        motor.set(motorPercentage);
    }

        //runs the motor in a "backwards" direction
        public void run(double _motorPercentage) {
            this.motorPercentage=_motorPercentage;
            motor.set(motorPercentage);
        }

    public void stopMotor()
    {
        motor.set(0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
        log();
    }

    public void log() {
        SmartDashboard.putNumber(this.toString() ,this.motorPercentage);
    }

    public String toString()
    {
        return "Unknown subsystem";
    }


}