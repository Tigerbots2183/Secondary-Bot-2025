// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class IntakeSpeed extends SubsystemBase {
  /** Creates a new IntakeSpeed. */
  SparkFlex topIntakeMotor = new SparkFlex(41, MotorType.kBrushless);
  SparkFlex sideIntakeMotor = new SparkFlex(42, MotorType.kBrushless);

  SparkFlexConfig intakeConfigurator = new SparkFlexConfig();


  public IntakeSpeed() {
    intakeConfigurator.smartCurrentLimit(50).idleMode(IdleMode.kCoast);
    topIntakeMotor.configure(intakeConfigurator, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    sideIntakeMotor.configure(intakeConfigurator, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIntakeSpeed(double speedTop, double speedSide){
    topIntakeMotor.set(speedTop);
    sideIntakeMotor.set(speedSide);
  }

}
