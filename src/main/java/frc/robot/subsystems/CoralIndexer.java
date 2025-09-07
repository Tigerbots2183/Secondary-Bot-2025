// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIndexer extends SubsystemBase {
  /** Creates a new CoralShooter. */

  SparkFlex indexCoralMotor = new SparkFlex(55, MotorType.kBrushless);

  SparkFlexConfig coralConfigurator = new SparkFlexConfig();

  public CoralIndexer() {
    coralConfigurator.smartCurrentLimit(50).idleMode(IdleMode.kCoast);

    indexCoralMotor.configure(coralConfigurator, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void indexAtSpeed(double speed){
    indexCoralMotor.set(-speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
