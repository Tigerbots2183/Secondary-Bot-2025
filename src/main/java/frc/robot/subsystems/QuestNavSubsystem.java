// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.io.File;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.VecBuilder;
import gg.questnav.questnav.PoseFrame;
import edu.wpi.first.math.geometry.Pose2d;
import gg.questnav.questnav.QuestNav;
import swervelib.SwerveDrive;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
public class QuestNavSubsystem extends SubsystemBase {
  /** Creates a new QuestNavSubsystem. */
  SwerveSubsystem swerveSubsystem;
  public QuestNavSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
  }

  QuestNav questNav = new QuestNav();

  @Override
  public void periodic() {
    questNav.commandPeriodic();
    
 
    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );

    if (questNav.isTracking()) {
      // Get the latest pose data frames from the Quest
      PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();

      // Loop over the pose data frames and send them to the pose estimator
      for (PoseFrame questFrame : questFrames) {
        // Get the pose of the Quest
        Pose2d questPose = questFrame.questPose();
        // Get timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get your robot pose
        Pose2d robotPose = questPose.transformBy(Constants.QuestNavConstants.ROBOT_TO_QUEST.inverse());

        // You can put some sort of filtering here if you would like!

        // Add the measurement to our estimator
        swerveSubsystem.getSwerveDrive().addVisionMeasurement(robotPose, timestamp, QUESTNAV_STD_DEVS);
      }
    }
  }
}
