/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants.ElevatorConstants;


public class Elevator extends UnifiedMotorController {
  /**
   * Creates a new ReplaceMeSubsystem.
   */
  private Victor locker;
  private double lockerMotorPercentage;
  public Elevator (){
    super();
    setConstant(ElevatorConstants.elevatorMotor);
    locker = new Victor(ElevatorConstants.lockerMotor);
  }

  @Override
  public void run(double _motorPercentage) {
    this.motorPercentage=_motorPercentage;
    lockerMotorPercentage=_motorPercentage;
    motor.set(motorPercentage);
    if(_motorPercentage<=0){
      locker.set(lockerMotorPercentage);
    }
  }
  public void lock(double lockerMotorPercentage){
    locker.set(lockerMotorPercentage);
  }

  public void log()
  {
    SmartDashboard.putNumber("Locker Speed", lockerMotorPercentage);
    SmartDashboard.putNumber("Elevator Speed", super.motorPercentage);
  }

  public double getLockerMotorPercentage() {
    return lockerMotorPercentage;
  }

  public void setLockerMotorPercentage(double lockerMotorPercentage) {
    this.lockerMotorPercentage = lockerMotorPercentage;
  }

  

}
