// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.POSES;
import frc.robot.Constants.StationPOSES;
import frc.robot.subsystems.Touchboard.*;
import frc.robot.commands.indexCommand;
import frc.robot.commands.AlignToPose;
import frc.robot.commands.IntakeSpeedCommand;
import frc.robot.commands.shooterCommand;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.CoralIndexer;
import frc.robot.subsystems.CoralPivoter;
import frc.robot.subsystems.CoralShooter;
import frc.robot.subsystems.IntakePivoter;
import frc.robot.subsystems.IntakeSpeed;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    private final CoralShooter coralShooter = new CoralShooter();
    private final CoralIndexer coralIndexer = new CoralIndexer();
    private final CoralPivoter coralPivoter = new CoralPivoter();
    private final IntakeSpeed intakeSpeed = new IntakeSpeed();
    private final IntakePivoter intakePivotor = new IntakePivoter();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    final CommandXboxController driverXbox = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve/neo"));

    private final OneShotButton PAbtn = new OneShotButton("PAbtn",
            () -> new AlignToPose(POSES.REEF_A, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PBbtn = new OneShotButton("PBbtn",
            () -> new AlignToPose(POSES.REEF_B, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PCbtn = new OneShotButton("PCbtn",
            () -> new AlignToPose(POSES.REEF_C, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PDbtn = new OneShotButton("PDbtn",
            () -> new AlignToPose(POSES.REEF_D, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PEbtn = new OneShotButton("PEbtn",
            () -> new AlignToPose(POSES.REEF_E, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PFbtn = new OneShotButton("PFbtn",
            () -> new AlignToPose(POSES.REEF_F, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PGbtn = new OneShotButton("PGbtn",
            () -> new AlignToPose(POSES.REEF_G, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PHbtn = new OneShotButton("PHbtn",
            () -> new AlignToPose(POSES.REEF_H, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PIbtn = new OneShotButton("PIbtn",
            () -> new AlignToPose(POSES.REEF_I, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PJbtn = new OneShotButton("PJbtn",
            () -> new AlignToPose(POSES.REEF_J, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PKbtn = new OneShotButton("PKbtn",
            () -> new AlignToPose(POSES.REEF_K, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    private final OneShotButton PLbtn = new OneShotButton("PLbtn",
            () -> new AlignToPose(POSES.REEF_L, drivebase).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    private final OneShotButton PLTbtm = new OneShotButton("LTbtn",
            () -> new AlignToPose(StationPOSES.Left_top_station, drivebase));
    private final OneShotButton PLMbtn = new OneShotButton("LMbtn",
            () -> new AlignToPose(StationPOSES.Left_mid_station, drivebase));
    private final OneShotButton PLBbtn = new OneShotButton("LBbtn",
            () -> new AlignToPose(StationPOSES.Left_bot_station, drivebase));
    private final OneShotButton PRTbtn = new OneShotButton("RTbtn",
            () -> new AlignToPose(StationPOSES.Right_top_station, drivebase));
    private final OneShotButton PRMbtn = new OneShotButton("RMbtn",
            () -> new AlignToPose(StationPOSES.Right_mid_station, drivebase));
    private final OneShotButton PRBbtn = new OneShotButton("RBbtn",
            () -> new AlignToPose(StationPOSES.Right_bot_station, drivebase));

    private final NumberComponent topSpeed = new NumberComponent("top");
    private final NumberComponent bLSpeed = new NumberComponent("bottomLeft");
    private final NumberComponent bRSpeed = new NumberComponent("bottomRight");
    private final NumberComponent indexerSpeed = new NumberComponent("indexSpeed");
    private final NumberComponent pivotAngle = new NumberComponent("pivotAngle");
    private final NumberComponent coralAngle = new NumberComponent("coralAngle");


    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> driverXbox.getLeftY() * -1,
            () -> driverXbox.getLeftX() * -1)
            .withControllerRotationAxis(driverXbox::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
            driverXbox::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -driverXbox.getLeftY(),
            () -> -driverXbox.getLeftX())
            .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2),
                    () -> Math.cos(driverXbox.getRawAxis(2) * Math.PI) * (Math.PI * 2))
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(
                    0));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        new QuestNavSubsystem(drivebase);
        configureBindings();
        DriverStation.silenceJoystickConnectionWarning(true);
        NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
     * an arbitrary predicate, or via the
     * named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for
     * {@link CommandXboxController
     * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
     * Flight joysticks}.
     */
    private void configureBindings() {
        
        new ToggleButton("Shoot", ()-> new shooterCommand(coralShooter, topSpeed.getValue(), bLSpeed.getValue(), bRSpeed.getValue()));

        new ActionButton("Index", ()-> new indexCommand(coralIndexer, indexerSpeed.getValue()));

        new ActionButton("startPivotDeg", () -> intakePivotor.setAngleCommand(Degrees.of(pivotAngle.getValue())));

        new ActionButton("startCoralDeg", () -> coralPivoter.setAngleCommand(Degrees.of(coralAngle.getValue())));

        new OneShotButton("Intake", ()-> intakePivotor.setAngleCommand(Degrees.of(0)).andThen(coralPivoter.setAngleCommand(Degrees.of(0)).andThen(new IntakeSpeedCommand(intakeSpeed, 1, 1))));

        Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
                driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
                driveDirectAngleKeyboard);

        if (RobotBase.isSimulation()) {
            drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }

        if (Robot.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1, 4),
                    Rotation2d.fromDegrees(90));
            // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
            driveDirectAngleKeyboard.driveToPose(() -> target,
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(5, 2)),
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(Units.degreesToRadians(360),
                                    Units.degreesToRadians(180))));
            driverXbox.start()
                    .onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                    () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

            // driverXbox.b().whileTrue(
            // drivebase.driveToPose(
            // new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
            // );

        }
        if (DriverStation.isTest()) {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

            driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
            driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.back().whileTrue(drivebase.centerModulesCommand());
            driverXbox.leftBumper().onTrue(Commands.none());
            driverXbox.rightBumper().onTrue(Commands.none());
        } else {
            driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
            driverXbox.start().whileTrue(Commands.none());
            driverXbox.back().whileTrue(Commands.none());
            driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            driverXbox.rightBumper().onTrue(Commands.none());
        }

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return drivebase.getAutonomousCommand("New Auto");
    }

    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
