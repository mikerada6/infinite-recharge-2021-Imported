/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants.EkimConstants;

/**
 * Add your docs here.
 */
public class Ekim extends UnifiedMotorController {
    private Victor back;
    public Ekim(){
        super();
        setConstant(EkimConstants.ekimMotor);
        back = new Victor(EkimConstants.backEkimMotor);
        back.setInverted(true);
    }

    @Override
    public void run(double _motorPercentage) {
        this.motorPercentage=_motorPercentage;
        motor.set(motorPercentage);
        back.set(_motorPercentage*1.5);
    }

    public void log()
    {
        SmartDashboard.putNumber("Ekim",this.motorPercentage);
    }
}
