// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be
 * declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED = Units.feetToMeters(14.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

  // public static final class AutonConstants
  // {
  //
  // public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0,
  // 0);
  // public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
  // }

  public static class POSES {
    public static final Pose2d RESET_POSE = new Pose2d(3.192, 4.025, new Rotation2d());

    // Blue branch poses
    public static Pose2d REEF_A = new Pose2d(3.40, 4.23, Rotation2d.fromDegrees(90)); // new Pose2d(3.18, 4.22, //
                                                                                      // Rotation2d.fromDegrees(90));
    public static Pose2d REEF_B = new Pose2d(3.21, 3.90, Rotation2d.fromDegrees(90));
    public static Pose2d REEF_C = new Pose2d(3.67, 2.98, Rotation2d.fromDegrees(150));
    public static Pose2d REEF_D = new Pose2d(3.91, 2.89, Rotation2d.fromDegrees(150));
    public static Pose2d REEF_E = new Pose2d(4.95, 2.82, Rotation2d.fromDegrees(-150));
    public static Pose2d REEF_F = new Pose2d(5.24, 2.98, Rotation2d.fromDegrees(-150));
    public static Pose2d REEF_G = new Pose2d(5.76, 3.81, Rotation2d.fromDegrees(-90));
    public static Pose2d REEF_H = new Pose2d(5.77, 4.16, Rotation2d.fromDegrees(-90));
    public static Pose2d REEF_I = new Pose2d(5.36, 4.84, Rotation2d.fromDegrees(-30));
    public static Pose2d REEF_J = new Pose2d(5.03, 5.19, Rotation2d.fromDegrees(-30));
    public static Pose2d REEF_K = new Pose2d(4.01, 5.27, Rotation2d.fromDegrees(30));
    public static Pose2d REEF_L = new Pose2d(3.75, 5.07, Rotation2d.fromDegrees(30));

  }

  public static class StationPOSES {
    public static final Pose2d RESET_POSE = new Pose2d(3.192, 4.025, new Rotation2d());

    public static final Pose2d Left_top_station = new Pose2d(1.55, 7.45, Rotation2d.fromDegrees(30));
    public static final Pose2d Left_mid_station = new Pose2d(1.18, 7.10, Rotation2d.fromDegrees(30));
    public static final Pose2d Left_bot_station = new Pose2d(0.62, 6.73, Rotation2d.fromDegrees(30));

    public static final Pose2d Right_top_station = new Pose2d(1.41, 0.73, Rotation2d.fromDegrees(150));
    public static final Pose2d Right_mid_station = new Pose2d(1.08, 1.19, Rotation2d.fromDegrees(150));
    public static final Pose2d Right_bot_station = new Pose2d(0.61, 1.55, Rotation2d.fromDegrees(150));
  }

  public static final class DrivebaseConstants {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants {

    // Joystick Deadband
    public static final double DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT = 6;
  }

  public static class QuestNavConstants {

    public static final Transform2d ROBOT_TO_QUEST = new Transform2d(0,0,Rotation2d.fromDegrees(90));
  }
}
