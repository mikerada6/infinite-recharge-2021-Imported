/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * Gains used in Motion Magic, to be adjusted accordingly
     * Gains(kp, ki, kd, kf, izone, peak output);
     */
    public static final Gains kGains = new Gains(0.2, 0.0, 0.0, 0.2, 0, 1.0);

    public static final class DriveConstants {

          // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = 0.22;
    public static final double kvVoltSecondsPerMeter = 1.98/39.3701;
    public static final double kaVoltSecondsSquaredPerMeter = 0.2/39.3701;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 8.5;

    public static final double kTrackwidthMeters = 23;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);



        public static final int RIGHTSON = 1;
        public static final int LEFTSON = 2;
        public static final int RIGHTFATHER = 3;
        public static final int LEFTFATHER = 4;
    
        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = true;
    
        public static final int kEncoderCPR = 4096;
        public static final double kWheelDiameterInches = 7.5;

        public static final double INCHESPERPULSE = (kWheelDiameterInches*Math.PI)/kEncoderCPR;
        public static final double kEncoderDistancePerPulse =
            // Assumes the encoders are directly mounted on the wheel shafts
            (kWheelDiameterInches * Math.PI) / (double) kEncoderCPR;

        public static double kMaxRPM = 5330;
        public static double kSensorUnitsPerRotation = kEncoderCPR;
        public static double kGearRatio=10.71;

        public static double magicNumber =  (kMaxRPM  / 600) * (kSensorUnitsPerRotation / kGearRatio);

        public static double linearize(final double x1, final double y1, final double x2, final double y2,
        final double input) {//TODO deadzone?
      final Double m = (y2 - y1) / (x2 - x1);
      final Double b = y1 - (-m * x1);
          return m*input+b;

      }

 public static final class ShooterConstants {

  public static final int shooterMotor = 5;
  public static double shooterSpeed=.25;
  public static int movingAverage= 25;
  
 }
 
 public static final class FlopperConstants{
   
  public static final int flopperMotor = 9;
  public static final int collecterMotor = 8;
  public static final int runTime =5000;
 }
 public static final class ElevatorConstants{
   
  public static final int elevatorMotor = 7;
  public static final double elevatorSpeed = .50;
  public static final int runTime =100;
public static final int lockerMotor = 6;
 }
/*
 public static final class ElevatorMoverConstants{

  public static final int elevatorMoverMotor = 0;
  public static final int runTime = 3500;
  
 }
*/
 public static final class EkimConstants{
  public static int ekimMotor = 4;
  public static int backEkimMotor = 3;
 }

 
 
 
 public static final class CollecterConstants{

  public static final int collecterMotor = 8;
 }

      public static final class AutoConstants {
        public static final double INCH = 1;
        public static final double FEET = 12;
        public static final double YARD = FEET*3;
        public static final double SECON = 1;
        public static final double kMaxSpeedMetersPerSecond = 3*39.3701;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3*39.3701;
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

      }
    }
  }
      

  

