// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.midi.Patch;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import java.util.function.Supplier;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToPose extends Command {
  private SwerveSubsystem s_Swerve;
  private Pose2d currentPose;
  private Command PathCommand = Commands.none();


  public AlignToPose(Pose2d currentPose, SwerveSubsystem s_Swerve) {
    PathCommand = Commands.none();
    this.currentPose = currentPose;
    this.s_Swerve = s_Swerve;
  }

  @Override
  public void initialize() {
    addRequirements(s_Swerve);

    PathConstraints constraints2 = new PathConstraints(
        4,
        2,
        4,
        3
    );

    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        PathCommand =AutoBuilder.pathfindToPose(
            FlippingUtil.flipFieldPose(currentPose),
            constraints2,
            0.00);

        PathCommand.schedule();

        return;
      }
    }

    PathCommand = AutoBuilder.pathfindToPose(
        currentPose,
        constraints2,
        0.00);
        PathCommand.schedule();


  }

  @Override
  public void execute() {
    // this.PathCommand.schedule();

  }

  @Override
  public void end(boolean interrupted) {
    PathCommand.cancel();
  }

  @Override
  public boolean isFinished() {
    return PathCommand.isFinished();
  }
}
