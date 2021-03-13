/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.analog.adis16448.frc.ADIS16448_IMU.IMUAxis;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IronMechEncoder;
import frc.robot.Constants.DriveConstants;



public class Drivetrain extends SubsystemBase {

  private final SpeedController m_leftMotor;
  private final SpeedController m_rightMotor;

  private final DifferentialDrive m_drive;

  // parent motors
  private final WPI_TalonSRX leftFather = new WPI_TalonSRX(DriveConstants.LEFTFATHER);
  private final WPI_TalonSRX rightFather = new WPI_TalonSRX(DriveConstants.RIGHTFATHER);

  // son motors
  private final WPI_VictorSPX leftSon = new WPI_VictorSPX(DriveConstants.LEFTSON);
  private final WPI_VictorSPX rightSon = new WPI_VictorSPX(DriveConstants.RIGHTSON);

  private final NeutralMode brakeMode = NeutralMode.Brake;

  private final IronMechEncoder leftEncoder;
  private final IronMechEncoder rightEncoder;

  private DifferentialDriveOdometry m_odometry;

  public static ADIS16448_IMU imu = new ADIS16448_IMU();



  /**
   * Creates a new Drivetrain.
   */

  public Drivetrain() {

    super();

    imu.calibrate();

    imu.setYawAxis(IMUAxis.kY);

    leftFather.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightFather.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    leftFather.configFactoryDefault();
    rightFather.configFactoryDefault();

    rightFather.setNeutralMode(brakeMode);
    leftFather.setNeutralMode(brakeMode);
    rightSon.setNeutralMode(brakeMode);
    leftSon.setNeutralMode(brakeMode);
    leftFather.setSensorPhase(true);
    rightFather.setSensorPhase(true);
    leftFather.setInverted(false);
    rightFather.setInverted(true);

    leftSon.follow(leftFather);
    rightSon.follow(rightFather);

    setTalon(leftFather);
    setTalon(rightFather);

    rightFather.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    rightFather.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    leftFather.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    leftFather.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)


    // Let's name the sensors on the LiveWindow

    m_leftMotor = new SpeedControllerGroup(leftFather, leftSon);
    m_rightMotor = new SpeedControllerGroup(rightFather, rightSon);
    m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
    m_drive.setSafetyEnabled(false);
    addChild("Drive", m_drive);

    leftEncoder = new IronMechEncoder(leftFather);
    rightEncoder = new IronMechEncoder(rightFather);

    leftEncoder.setDistancePerPulse(DriveConstants.METERSPERPULSE);
    rightEncoder.setDistancePerPulse(DriveConstants.METERSPERPULSE);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(imu.getGyroAngleY()));

  }

    /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(Rotation2d.fromDegrees(imu.getGyroAngleY()), leftEncoder.getDistance(),
                      rightEncoder.getDistance());
    log();
  }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry( Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(imu.getGyroAngleY()));
  }

  /**
   * The log method puts interesting information to the SmartDashboard.
   */
  public void log() {
    SmartDashboard.putNumber("imuAngle", imu.getAngle());
    SmartDashboard.putNumber("pitch", imu.getGyroAngleX());
    SmartDashboard.putNumber("yaw", imu.getGyroAngleY());
    SmartDashboard.putNumber("roll", imu.getGyroAngleZ());
    


    SmartDashboard.putNumber("rightVelocity",rightEncoder.getRate());
    SmartDashboard.putNumber("leftVelocity", leftEncoder.getRate());

    SmartDashboard.putNumber("rightDistance",rightEncoder.getDistance());
    SmartDashboard.putNumber("leftDistance", leftEncoder.getDistance());

  }

  

    /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    System.out.println("Left: " + leftVolts +"\tRight:" + rightVolts);
    leftFather.setVoltage(leftVolts);
    rightFather.setVoltage(rightVolts);
  }

  /**
   * Tank style driving for the DriveTrain.
   *
   * @param left  Speed in range [-1,1]
   * @param right Speed in range [-1,1]
   */
  public void drive(final double left, final double right) {
    m_drive.tankDrive(left, right);
  }

    /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }

  public Object arcadeDrive(double fwd, double rot) {
    // rightFather.set(ControlMode.PercentOutput,fwd+rot);
    // leftFather.set(ControlMode.PercentOutput,fwd-rot);
    if (Math.abs(fwd) <= .01) {
      fwd = 0;
    }
    if (Math.abs(rot) <= .01) {
      rot = 0;
    }
    m_drive.arcadeDrive(fwd, -1 * rot);
    // m_drive.tankDrive(left, right);
    return null;
  }

  public double getAverageEncoderDistance() {
    return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    imu.reset();
  }

  public double getHeading() {
    return imu.getGyroAngleY();
  }

  public double getTurnRate() {
    return -imu.getRate();
  }

  public void setTalon(final WPI_TalonSRX _talon) {

    /* Set relevant frame periods to be at least as fast as periodic rate */
    _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
    _talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

    /* Set the peak and nominal outputs */
    _talon.configNominalOutputForward(0, Constants.kTimeoutMs);
    _talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
    _talon.configPeakOutputForward(1, Constants.kTimeoutMs);
    _talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    /* Set Motion Magic gains in slot0 - see documentation */
    _talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
    _talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
    _talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
    _talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
    _talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
    _talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
    _talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

    /* Zero the sensor once on robot boot up */
    _talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
  }


  private void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

}
