/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Welcome to Gary ver1.0.0

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Ekim;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.AutoConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.FineDrive;
import frc.robot.commands.ReadData;
import frc.robot.commands.StopEverything;
import frc.robot.commands.TurnOnMotor;
import frc.robot.subsystems.Flopper;
/**
 * Add your docs here.
 */
public class RobotContainer {

    
    //all the subsstem
    private final Drivetrain m_robotDrive = new Drivetrain();
    private final Shooter m_shooter = new Shooter();
    private final Flopper flopperSubsystem = new Flopper();
    private final Elevator elvatorSubsystem = new Elevator();
    private final Collector collectorSubsystem = new Collector();
    private final Ekim ekimSubsystem = new Ekim();

    private final Joystick driverLeftStick = new Joystick(0);
    private final Joystick driverRightStick = new Joystick(1);
    private final Joystick copilot = new Joystick (2);

    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public RobotContainer() {

        SmartDashboard.putData("Stop Everything", new StopEverything(ekimSubsystem, flopperSubsystem, elvatorSubsystem,collectorSubsystem, m_shooter ));


        // Configure default commands
        m_robotDrive.setDefaultCommand(new Drive(() -> driverLeftStick.getX(Hand.kLeft),
                () -> driverRightStick.getY(Hand.kRight), m_robotDrive));
        m_shooter.setDefaultCommand(new ReadData(m_shooter));

        // Configure the button bindings
        configureButtonBindings();

        driverLeftStick.getZ();

            // Show what command your subsystem is running on the SmartDashboard
    SmartDashboard.putData(m_robotDrive);
    SmartDashboard.putData(m_shooter);
    SmartDashboard.putData(flopperSubsystem);
    SmartDashboard.putData(elvatorSubsystem);
    SmartDashboard.putData(collectorSubsystem);
    SmartDashboard.putData(ekimSubsystem);

    autoChooser.setDefaultOption("Simple","A");
    autoChooser.addOption("Wait 1","B");
    autoChooser.addOption("Wait 2","C");
    autoChooser.addOption("Wait 3","D");
    autoChooser.addOption("Wait 4","E");
    autoChooser.addOption("Wait 5","F");
    SmartDashboard.putData("Auto Options", autoChooser);
    
    
    }

    public String getAuto()
    {
        return autoChooser.getSelected();
    }

    public Command getAutonomousCommand(Trajectory trajectory) {
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                       DriveConstants.kvVoltSecondsPerMeter,
                                       DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    //                          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(3, 1),
    //         new Translation2d(-3, 3)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(0, 6, new Rotation2d(0)),
    //     // Pass config
    //     config
    // );



    RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory,
        () -> m_robotDrive.getPose(),
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts,
                                   DriveConstants.kvVoltSecondsPerMeter,
                                   DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts,
        m_robotDrive
    );

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        //copilot buttons
        final JoystickButton shooterOn = new JoystickButton(copilot, 1);
        final JoystickButton ekimDown = new JoystickButton(copilot, 2); 
        final JoystickButton ekimOn = new JoystickButton(copilot, 3);
        final JoystickButton ekimCollector = new JoystickButton(copilot, 5);
        final JoystickButton elevatorDown = new JoystickButton(copilot, 6);
        final JoystickButton elevatorUp = new JoystickButton(copilot, 7);
        final JoystickButton stop2 = new JoystickButton(copilot, 9);
        final JoystickButton coCollectorIn = new JoystickButton(copilot, 10);
        final JoystickButton coCollectorOut = new JoystickButton(copilot, 11);
        
        //driver left buttons
        final JoystickButton stop = new JoystickButton(driverLeftStick, 1);
        final JoystickButton shootBack = new JoystickButton(driverLeftStick, 8);

        //driver right buttons
        final JoystickButton flopperOn = new JoystickButton(driverRightStick, 1);     
        final JoystickButton activateFineDrive = new JoystickButton(driverRightStick, 2);  



        //copilot actions
        shooterOn.toggleWhenPressed(new TurnOnMotor(m_shooter,-1));
        ekimDown.whileHeld(new TurnOnMotor(ekimSubsystem,-.5));
        ekimOn.whileHeld(new TurnOnMotor(ekimSubsystem,.5));
        ekimCollector.whileHeld(new TurnOnMotor(ekimSubsystem,.5).alongWith(new TurnOnMotor(collectorSubsystem,1)));
        elevatorDown.whileHeld(new TurnOnMotor(elvatorSubsystem,-1));
        elevatorUp.whileHeld(new TurnOnMotor(elvatorSubsystem,1));
        coCollectorIn.whileHeld(new TurnOnMotor(collectorSubsystem,.75));
        coCollectorOut.whileHeld(new TurnOnMotor(collectorSubsystem,-.75));

        //driver left commands
        shootBack.toggleWhenPressed(new TurnOnMotor(m_shooter,-.5));

        //driver right commands
        flopperOn.whileHeld(new TurnOnMotor(flopperSubsystem,.5));
        activateFineDrive.whileHeld(new FineDrive(m_robotDrive,driverRightStick));

        //stops
        stop.toggleWhenPressed(new StopEverything(ekimSubsystem, flopperSubsystem, elvatorSubsystem,collectorSubsystem, m_shooter ));
        stop2.toggleWhenPressed(new StopEverything(ekimSubsystem, flopperSubsystem, elvatorSubsystem,collectorSubsystem, m_shooter ));

    }
}
