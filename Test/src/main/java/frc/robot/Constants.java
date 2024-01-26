// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
//Trackwidth is the distance between front 2 modules
//wheebase is the distance between the front to the back Modules
  public static final double trackWidth = Units.inchesToMeters(16.316);
  public static final double wheelBase = Units.inchesToMeters(22.655);


  //CHANGE IDS FOR ALL MOTORS AND ENCODERS
  public static final int FrontLeftDriveID = 0;
  public static final int FrontRightDriveID = 0;
  public static final int BackLeftDriveID = 0;
  public static final int BackRightDriveID = 0;

  public static final int FrontLeftSteerID = 0;
  public static final int FrontRightSteerID = 0;
  public static final int BackLeftSteerID = 0;
  public static final int BackRightSteerID = 0;

  public static final int FrontLeftEncoderID = 0;
  public static final int FrontRightEncoderID = 0;
  public static final int BackLeftEncoderID = 0;
  public static final int BackRightEncoderID = 0;

  public static final double FrontLeftOffset = 0;
  public static final double FrontRightOffset = 0;
  public static final double BackLeftOffset = 0;
  public static final double BackRightOffset = 0;

  public static final boolean FrontLeftInv = false;
  public static final boolean FrontRightInv = true;
  public static final boolean BackLeftInv = false;
  public static final boolean BackRightInv = true;

  public static final int PigeonID = 0;

  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(wheelBase/2, -trackWidth/2), // Front Right
    new Translation2d(wheelBase/2, trackWidth/2),  // FrontLeft
    new Translation2d(-wheelBase/2, -trackWidth/2), // Back Right
    new Translation2d(-wheelBase/2, trackWidth/2) // Rear Left
  );

  public static class ModConstants{
    // same Kp values for all modules, Might need to adjust
    public static final double KP = 0.4;
    public static final double KI = 0;
    public static final double KD = 0;
  }

  public static class OperatorConstants {
    public static final int DriverControllerPort = 0;
    public static final double DriverDeadband = 0.05;

    public static final int OperatorControllerPort = 1;
  }
}
