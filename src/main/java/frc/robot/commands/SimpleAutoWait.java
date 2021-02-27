/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ekim;
import frc.robot.subsystems.Flopper;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SimpleAutoWait extends SequentialCommandGroup {
  /**
   * Creates a new SimpleAutoWait.
   */
  public SimpleAutoWait(Flopper flopper, Drivetrain drivetrain, Shooter shooter, Ekim ekim, int wait) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super();
        // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new TurnOnMotor(flopper,-.25).withTimeout(.75),
      new ParallelCommandGroup(
        new DriveStraight(drivetrain).withTimeout(3.75),
        new TurnOnMotor(shooter, .95),
        new SequentialCommandGroup(
            new WaitCommand(3.9),
            new WaitCommand(wait),
            new TurnOnMotor(ekim,.35)
        )
    ));
  }
  }
