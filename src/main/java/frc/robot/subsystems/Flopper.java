/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants.FlopperConstants;

/**
 * Add your docs here.
 */
public class Flopper extends UnifiedMotorController{

    public Flopper()
    {
        super();
        super.setConstant(FlopperConstants.flopperMotor);
    }

    public void log()
    {
        SmartDashboard.putNumber("Flopper", super.motorPercentage);
    }
}
