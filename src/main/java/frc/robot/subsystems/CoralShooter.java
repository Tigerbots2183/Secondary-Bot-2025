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

public class CoralShooter extends SubsystemBase {
  /** Creates a new CoralShooter. */

  SparkFlex topCoralMotor = new SparkFlex(52, MotorType.kBrushless);
  //Looking at front of coral shooter
  SparkFlex leftCoralMotor = new SparkFlex(51, MotorType.kBrushless);
  SparkFlex rightCoralMotor = new SparkFlex(53, MotorType.kBrushless);

  SparkFlexConfig coralConfigurator = new SparkFlexConfig();

  public CoralShooter() {
    coralConfigurator.smartCurrentLimit(50).idleMode(IdleMode.kCoast);

    topCoralMotor.configure(coralConfigurator, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    leftCoralMotor.configure(coralConfigurator, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    rightCoralMotor.configure(coralConfigurator, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void shootAtSpeed(double speedTop, double speedBL, double speedBR){
    topCoralMotor.set(speedTop);
    leftCoralMotor.set(speedBL);
    rightCoralMotor.set(-speedBR);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
