// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;

import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class CoralPivoter extends SubsystemBase {
  /** Creates a new CoralPivoter. */

  SparkMax coralPivotSpark = new SparkMax(50, MotorType.kBrushless);
  private SmartMotorControllerConfig smcoralConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  .withSimClosedLoopController(50, 0, 0, DegreesPerSecond.of(90), DegreesPerSecondPerSecond.of(45))
  // Feedforward Constants
  .withFeedforward(new ArmFeedforward(0, 0, 0))
  .withSimFeedforward(new ArmFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("CoralPivotMotor", TelemetryVerbosity.HIGH)
  // Gearing from the motor rotor to final shaft.
  // In this example gearbox(3,4) is the same as gearbox("3:1","4:1") which corresponds to the gearbox attached to your motor.
  .withGearing(SmartMechanism.gearing(SmartMechanism.gearbox(20)))
  // Motor properties to prevent over currenting.
  .withMotorInverted(false)
  .withIdleMode(MotorMode.BRAKE)
  .withStatorCurrentLimit(Amps.of(40))
  .withClosedLoopRampRate(Seconds.of(0.25))
  .withOpenLoopRampRate(Seconds.of(0.25));

  private SmartMotorController coralSmartMotorController = new SparkWrapper(coralPivotSpark, DCMotor.getNEO(1), smcoralConfig);


  private ArmConfig coralCfg = new ArmConfig(coralSmartMotorController)
  // Soft limit is applied to the SmartMotorControllers PID
  .withSoftLimits(Degrees.of(-30), Degrees.of(110))
  // Hard limit is applied to the simulation.

  .withHardLimit(Degrees.of(-30), Degrees.of(110))
  // Starting position is where your arm starts
  .withStartingPosition(Degrees.of(90))
  // Length and mass of your arm for sim.
  .withLength(Inches.of(13))
  .withMass(Pounds.of(5.5))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("coralPivot", TelemetryVerbosity.HIGH);

  private Arm coralPivot = new Arm(coralCfg);

  public Command setAngleCommand(Angle angle) {
    return coralPivot.setAngle(angle);
  }

  // public Command set(double speed){return intakePivot.set(speed);}

  public Command sysId() { return coralPivot.sysId(Volts.of(7), Volts.of(2).per(Second), Seconds.of(4));}


  public CoralPivoter() {}



  public double getArmPosition(){
    return coralPivot.getAngle().in(Degrees);
  }


  @Override
  public void periodic() {
    coralPivot.updateTelemetry();
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    coralPivot.simIterate();
  }
}
