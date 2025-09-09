// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakePivoter;

import static edu.wpi.first.units.Units.Degrees;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.IntSequenceGenerator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePivotCommand extends Command {
  /** Creates a new IntakePivotCommand. */
  IntakePivoter intakeP;
  Command angleCommand;
  double angle;

  public IntakePivotCommand(IntakePivoter intakeP, double angle) {
    addRequirements(intakeP);

    this.intakeP = intakeP;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    angleCommand = intakeP.setAngleCommand(Degrees.of(angle));

    angleCommand.schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (angle < 0) {

      return angle + 1 > intakeP.getArmPosition();
    } else if (angle == 0) {
      return false;
      // return intakeP.getArmPosition() - 1 < 0 && intakeP.getArmPosition() + 1 > 0;
    } else {

      return angle - 1 < intakeP.getArmPosition();
    }
  }
}
