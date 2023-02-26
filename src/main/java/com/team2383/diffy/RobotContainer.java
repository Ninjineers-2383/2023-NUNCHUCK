// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.diffy;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.diffy.autos.FullAutoCommand;
import com.team2383.diffy.commands.PaddleCommandPosition;
import com.team2383.diffy.commands.FeederCommand;
import com.team2383.diffy.commands.JoystickDriveCommand;
import com.team2383.diffy.commands.PaddleCommand;
import com.team2383.diffy.commands.pinkArm.PinkArmPresetCommand;
import com.team2383.diffy.commands.pinkArm.velocity.PivotVelocityCommand;
import com.team2383.diffy.commands.pinkArm.velocity.TelescopeVelocityCommand;
import com.team2383.diffy.commands.pinkArm.velocity.WristVelocityCommand;
import com.team2383.diffy.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.diffy.subsystems.paddle.PaddleSubsystem;
import com.team2383.diffy.subsystems.pinkArm.PinkArmSimSubsystem;
import com.team2383.diffy.subsystems.pinkArm.feeder.FeederSubsystem;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;
import com.team2383.diffy.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.diffy.commands.pinkArm.position.PositionConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private final GenericHID m_driverController = new GenericHID(0);
    private final XboxController m_operatorController = new XboxController(2);

    // Power and suppliers are defined here
    private final Supplier<Translation2d> m_drive = () -> new Translation2d(
            MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX), .1),
            MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY), .1));

    private final Supplier<Rotation2d> m_driveOmega = () -> Rotation2d
            .fromDegrees(90 * m_driverController.getRawAxis(0));

    // private final BooleanSupplier m_fieldCentric = () ->
    // !(m_driverController.getRawButton(5));
    private final JoystickButton m_paddleFeed = new JoystickButton(m_driverController, 5);
    private final IntSupplier m_povSupplier = () -> -1;

    private final DoubleSupplier m_intake = () -> MathUtil
            .applyDeadband(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(2), .1);

    private final Supplier<Rotation2d> m_pivot = () -> Rotation2d
            .fromDegrees(90 * m_operatorController.getRawAxis(5));
    private final DoubleSupplier m_extension = () -> 7 * m_operatorController.getRawAxis(1);
    private final Supplier<Rotation2d> m_wrist = () -> Rotation2d
            .fromDegrees(90 * (m_operatorController.getRawAxis(3) - m_operatorController.getRawAxis(2)));

    private final Trigger m_presetFeedCube = new Trigger(() -> SmartDashboard.getBoolean("Feed Cube", false));
    private final Trigger m_presetFeedCone = new Trigger(() -> SmartDashboard.getBoolean("Feed Cone", false));
    private final Trigger m_presetShootLow = new Trigger(() -> SmartDashboard.getBoolean("Shoot Low", false));
    private final Trigger m_presetShootHigh = new Trigger(() -> SmartDashboard.getBoolean("Shoot High", false));
    private final Trigger m_presetShootMid = new Trigger(() -> SmartDashboard.getBoolean("Shoot Mid", false));

    private final Trigger m_resetPosition = new Trigger(() -> SmartDashboard.getBoolean("Reset Position", false));

    private final Trigger m_zeroArm = new Trigger(() -> SmartDashboard.getBoolean("Zero Arm", false));

    private final Trigger m_travel = new Trigger(() -> SmartDashboard.getBoolean("Travel Pos", false));
    private final Trigger m_resetHeading = new Trigger(() -> SmartDashboard.getBoolean("Reset Heading", false));

    private final Trigger m_paddlePreset = new Trigger(() -> SmartDashboard.getBoolean("Paddle Preset", false));

    

    // The robot's subsystems and commands are defined here...

    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(DataLogManager.getLog());
    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(DataLogManager.getLog());
    private final PaddleSubsystem m_dickSubsystem = new PaddleSubsystem();
    private Supplier<Rotation2d> m_pivotSupplier;
    private final TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem(m_pivotSupplier);
    private final WristSubsystem m_wristSubsystem = new WristSubsystem(m_pivotSupplier);
    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(m_telescopeSubsystem::getExtensionInches);

    private final PinkArmSimSubsystem pinkArmSim = new PinkArmSimSubsystem(m_pivotSubsystem,
            m_telescopeSubsystem, m_wristSubsystem);

    // Commands are defined here

    private final JoystickDriveCommand m_driveCommand = new JoystickDriveCommand(m_drivetrainSubsystem,
            m_drive,
            m_driveOmega, () -> true, m_povSupplier);
    private final FeederCommand m_feederCommand = new FeederCommand(m_feederSubsystem, m_intake);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.

    HashMap<String, Command> autoEventMap = new HashMap<>() {
        {
            put("Auto Log", new PrintCommand("Auto Event: log"));

            put("Feed Cone", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.FEED_UPRIGHT_CONE),
                    new FeederCommand(m_feederSubsystem, () -> 1).withTimeout(0.7)));
            put("Feed Cube", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.FEED_CONE_POS),
                    new FeederCommand(m_feederSubsystem, () -> 1)
                            .withTimeout(0.7)));

            put("Score Cone Low", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.LOW_SCORE_POS),
                    new FeederCommand(m_feederSubsystem, () -> -1)
                            .withTimeout(0.7)));
            put("Score Cone Mid", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.MID_SCORE_POS),
                    new FeederCommand(m_feederSubsystem, () -> -1)
                            .withTimeout(0.7)));
            put("Score Cone High", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.HIGH_SCORE_POS),
                    new FeederCommand(m_feederSubsystem, () -> -1)
                            .withTimeout(0.7)));

            put("Score Cube Low", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.LOW_SCORE_POS),
                    new FeederCommand(m_feederSubsystem, () -> -0.6)
                            .withTimeout(0.7)));
            put("Score Cube Mid", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.MID_SCORE_POS),
                    new FeederCommand(m_feederSubsystem, () -> -0.6)
                            .withTimeout(0.7)));
            put("Score Cube High", new SequentialCommandGroup(
                    new PinkArmPresetCommand(
                            m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                            PositionConstants.HIGH_SCORE_POS),
                    new FeederCommand(m_feederSubsystem, () -> -0.6)
                            .withTimeout(0.7)));
        }
    };

    SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
            m_drivetrainSubsystem::getPose,
            m_drivetrainSubsystem::forceOdometry,
            m_drivetrainSubsystem.m_kinematics,
            new PIDConstants(5, 0, 0),
            new PIDConstants(0.5, 0, 0),
            m_drivetrainSubsystem::setModuleStates,
            autoEventMap,
            true,
            m_drivetrainSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        pinkArmSim.init();
        // Configure the button bindings
        m_telescopeSubsystem.setPivotAngle(m_pivotSubsystem::getAngle);
        m_wristSubsystem.setPivotAngle(m_pivotSubsystem::getAngle);

        publishDashboard("");
        configureButtonBindings();
        configureDefaultCommands();

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        LiveWindow.enableAllTelemetry();

        setAutoCommands();
    }

    private void publishDashboard(String key) {
        String[] keys = new String[] { "Feed Cube", "Feed Cone", "Shoot Low", "Shoot High", "Shoot Mid",
                "Reset Position", "Zero Arm", "Reset Heading", "Travel Pos"};
        for (String keyz : keys) {
            if (!key.equals(keyz)) {
                SmartDashboard.putBoolean(keyz, false);
            }
        }
    }

    private void configureButtonBindings() {
        m_presetFeedCube.onTrue(
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.FEED_CUBE_POS).andThen(() -> publishDashboard("Feed Cube")));

        m_presetFeedCone.onTrue(
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.FEED_CONE_POS).andThen(() -> publishDashboard("Feed Cone")));

        m_presetShootLow.onTrue(
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.LOW_SCORE_POS).andThen(() -> publishDashboard("Shoot Low")));

        m_presetShootMid.onTrue(
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.MID_SCORE_POS).andThen(() -> publishDashboard("Shoot High")));

        m_presetShootHigh.onTrue(
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem,
                        m_wristSubsystem,
                        PositionConstants.HIGH_SCORE_POS).andThen(() -> publishDashboard("Shoot Mid")));

        m_resetPosition.onTrue(new InstantCommand(m_telescopeSubsystem::resetPosition)
                .andThen(() -> publishDashboard("Reset Position")));

        m_resetHeading.onTrue(new InstantCommand(m_drivetrainSubsystem::resetHeading)
                .andThen(() -> publishDashboard("Reset Heading")));

        m_zeroArm.onTrue(
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.ZERO_POS).andThen(() -> publishDashboard("Zero Arm")));

        m_paddlePreset.onTrue(new PaddleCommandPosition(m_dickSubsystem, Rotation2d.fromDegrees(180)))
                .onFalse(new PaddleCommandPosition(m_dickSubsystem, Rotation2d.fromDegrees(0)));

        m_travel.onTrue(
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem, 
                        PositionConstants.TRAVEL_POS).andThen(() -> publishDashboard("Travel Pos")));

    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);
        m_feederSubsystem.setDefaultCommand(m_feederCommand);
        m_telescopeSubsystem.setDefaultCommand(new TelescopeVelocityCommand(m_telescopeSubsystem, m_extension));
        m_pivotSubsystem.setDefaultCommand(new PivotVelocityCommand(m_pivotSubsystem, m_pivot));
        m_wristSubsystem.setDefaultCommand(new WristVelocityCommand(m_wristSubsystem, m_wrist));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return new FullAutoCommand(m_drivetrainSubsystem, "Forward", m_autoBuilder);
        // return new FullAutoCommand(m_drivetrainSubsystem, "TwoCones", m_autoBuilder);
        // return autoChooser.getSelected();
        return null;// m_pinkArmAutoCommand;
    }

    private void setAutoCommands() {
        Command forwardTest = new FullAutoCommand(m_drivetrainSubsystem, "Forward",
                m_autoBuilder);
        Command twoConeAuto = new FullAutoCommand(m_drivetrainSubsystem, "TwoCones",
                m_autoBuilder);

        Command nullAuto = null;

        autoChooser.setDefaultOption("Two Cone Path Auto", twoConeAuto);
        autoChooser.addOption("Forward Test Auto", forwardTest);
        autoChooser.addOption("No Auto :(", nullAuto);
    }
}
