// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.nunchuck;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.nunchuck.autos.CubeMobilityAuto;
import com.team2383.nunchuck.autos.Engage;
import com.team2383.nunchuck.autos.FullAutoCommand;
import com.team2383.nunchuck.autos.ScorePreloadHigh;
import com.team2383.nunchuck.autos.ScorePreloadMid;
import com.team2383.nunchuck.commands.FeederCommand;
import com.team2383.nunchuck.commands.JoystickDriveHeadingLock;
import com.team2383.nunchuck.commands.pinkArm.PinkArmPresetCommand;
import com.team2383.nunchuck.commands.pinkArm.velocity.PivotVelocityCommand;
import com.team2383.nunchuck.commands.pinkArm.velocity.TelescopeVelocityCommand;
import com.team2383.nunchuck.commands.pinkArm.velocity.WristVelocityCommand;
import com.team2383.nunchuck.commands.pinkArm.zero.ZeroTelescope;
import com.team2383.nunchuck.helpers.ButtonBoardButtons;
import com.team2383.nunchuck.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.PinkArmSimSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.feeder.FeederIOFalcon500;
import com.team2383.nunchuck.subsystems.pinkArm.feeder.FeederSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.pivot.PivotIOSparkMax;
import com.team2383.nunchuck.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.telescope.TelescopeIOSparkMax;
import com.team2383.nunchuck.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.nunchuck.subsystems.pinkArm.wrist.WristIOTalonSRX;
import com.team2383.nunchuck.subsystems.pinkArm.wrist.WristSubsystem;
import com.team2383.nunchuck.commands.pinkArm.position.PivotPositionCommand;
import com.team2383.nunchuck.commands.pinkArm.position.PositionConstants;
import com.team2383.nunchuck.commands.pinkArm.position.PositionConstants.PinkPositions;

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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
            .fromDegrees(100 * MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega), 0.1));

    private final BooleanSupplier m_fieldCentric = () -> !(m_driverController.getRawButton(Constants.OI.FieldCentric));
    private final IntSupplier m_povSupplier = () -> -1;

    private final DoubleSupplier m_intake = () -> MathUtil
            .applyDeadband(m_driverController.getRawAxis(Constants.OI.IntakeIn) / 2.0
                    - (m_driverController.getRawAxis(Constants.OI.IntakeOut)
                            + Math.signum(
                                    MathUtil.applyDeadband(m_driverController.getRawAxis(Constants.OI.IntakeOut), 0.1)
                                            * 0.2))
                    + 0.2, .1);

    private final Supplier<Rotation2d> m_pivot = () -> Rotation2d
            .fromDegrees(90 * MathUtil.applyDeadband(m_operatorController.getRawAxis(5), 0.1));
    private final DoubleSupplier m_extension = () -> 12
            * MathUtil.applyDeadband(m_operatorController.getRawAxis(1), 0.1);
    private final Supplier<Rotation2d> m_wrist = () -> Rotation2d
            .fromDegrees(90 * MathUtil
                    .applyDeadband(m_operatorController.getRawAxis(3) - m_operatorController.getRawAxis(2), 0.1));

    private final Trigger m_resetPosition = ButtonBoardButtons.makeNormieButton("Reset Position");
    private final Trigger m_resetHeading = ButtonBoardButtons.makeNormieButton("Reset Heading")
            .or(() -> m_driverController.getRawButton(Constants.OI.ResetHeading));
    // private final Trigger m_paddlePreset =
    // ButtonBoardButtons.makeNormieButton("Paddle Preset")
    // .or(new JoystickButton(m_driverController, 6));
    // private final Trigger m_paddleHome =
    // ButtonBoardButtons.makeNormieButton("Paddle Home")
    // .or(new JoystickButton(m_driverController, 5));

    // private final Trigger m_paddleRetrieve =
    // ButtonBoardButtons.makeNormieButton("Retrieve From Paddle");

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(DataLogManager.getLog());
    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(new FeederIOFalcon500());
    // private final PaddleSubsystem m_dickSubsystem = new PaddleSubsystem();
    private Supplier<Rotation2d> m_pivotSupplier;
    private final TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem(new TelescopeIOSparkMax(), m_pivotSupplier);
    private final WristSubsystem m_wristSubsystem = new WristSubsystem(new WristIOTalonSRX(), m_pivotSupplier);
    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(new PivotIOSparkMax(), m_telescopeSubsystem::getExtensionInches);

    private final PinkArmSimSubsystem pinkArmSim = new PinkArmSimSubsystem(m_pivotSubsystem,
            m_telescopeSubsystem, m_wristSubsystem);

    // Commands are defined here
    private final JoystickDriveHeadingLock m_driveCommand = new JoystickDriveHeadingLock(
            m_drivetrainSubsystem,
            m_drive,
            m_driveOmega,
            m_fieldCentric,
            m_povSupplier);
    private final FeederCommand m_feederCommand = new FeederCommand(m_feederSubsystem, m_intake);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> autoHashMap = new HashMap<>() {
        {
            put("Auto Log", new PrintCommand("Auto Event: log"));

            put("Cube", new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                    PositionConstants.FEED_CUBE_POS));

            put("Travel",
                    new SequentialCommandGroup(
                            new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                                    PositionConstants.TRAVEL_POS),
                            new FeederCommand(m_feederSubsystem, () -> 0).withTimeout(0.7)));

            put("Safety", new PivotPositionCommand(m_pivotSubsystem, Rotation2d.fromDegrees(-60)));

            put("Intake", new FeederCommand(m_feederSubsystem, () -> 1));
            put("Outake", new FeederCommand(m_feederSubsystem, () -> -0.5));
            put("OutakeFull", new FeederCommand(m_feederSubsystem, () -> -1));

            put("Feeder Off", new FeederCommand(m_feederSubsystem, () -> 0).withTimeout(0));

            put("Low", new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                    PositionConstants.LOW_SCORE_BACK));

            put("Mid", new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                    PositionConstants.MID_SCORE_BACK));

            put("High", new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                    PositionConstants.HIGH_SCORE_BACK));
        }
    };

    public SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_drivetrainSubsystem::getPose,
            m_drivetrainSubsystem::forceOdometry,
            m_drivetrainSubsystem.m_kinematics,
            new PIDConstants(5, 0, 0),
            new PIDConstants(4, 0, 0),
            m_drivetrainSubsystem::setModuleStates,
            autoHashMap,
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

        configureButtonBindings();
        configureDefaultCommands();

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        LiveWindow.enableAllTelemetry();

        setAutoCommands();
    }

    private void configureButtonBindings() {
        ButtonBoardButtons.instantiatePositionalButtons(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem);
        m_resetPosition.onTrue(new ZeroTelescope(m_telescopeSubsystem));
        m_resetHeading.onTrue(new InstantCommand(m_drivetrainSubsystem::resetHeading));

        // m_paddlePreset.onTrue(new PaddleCommandPosition(m_dickSubsystem,
        // Rotation2d.fromDegrees(165)))
        // .onFalse(new PaddleCommandPosition(m_dickSubsystem,
        // Rotation2d.fromDegrees(70)));
        // m_paddleHome.onTrue(new PaddleCommandPosition(m_dickSubsystem,
        // Rotation2d.fromDegrees(-5)));

        // m_paddleRetrieve.onTrue(new TravelCommand(m_pivotSubsystem,
        // m_telescopeSubsystem, m_wristSubsystem,
        // m_dickSubsystem, m_feederSubsystem));

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
        return autoChooser.getSelected();
    }

    private void setAutoCommands() {
        Command score_preload_high = new ScorePreloadHigh(m_drivetrainSubsystem, m_telescopeSubsystem, m_pivotSubsystem,
                m_wristSubsystem, m_feederSubsystem);

        Command score_preload_mid = new ScorePreloadMid(m_drivetrainSubsystem, m_telescopeSubsystem, m_pivotSubsystem,
                m_wristSubsystem, m_feederSubsystem);

        Command cube_mobility = new CubeMobilityAuto(m_drivetrainSubsystem, m_telescopeSubsystem, m_pivotSubsystem,
                m_wristSubsystem, m_feederSubsystem, autoBuilder);

        Command cone_cube_2 = new SequentialCommandGroup(
                new ZeroTelescope(m_telescopeSubsystem),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        new PinkPositions("", PositionConstants.HIGH_SCORE_BACK.pivot, 0, new Rotation2d(0))),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.HIGH_SCORE_BACK),
                new FeederCommand(m_feederSubsystem, () -> -1).withTimeout(0.4),
                new FullAutoCommand(m_drivetrainSubsystem, "ConeCube2", autoBuilder));

        Command center_one = new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrainSubsystem.forceHeading(Rotation2d.fromDegrees(180))),
                new ZeroTelescope(m_telescopeSubsystem),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.HIGH_SCORE_FRONT),
                new FeederCommand(m_feederSubsystem, () -> -1).withTimeout(0.4),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.ZERO_POS),
                new FullAutoCommand(m_drivetrainSubsystem, "CenterOne", autoBuilder),
                new Engage(m_drivetrainSubsystem, false));

        Command dirty_cube = new SequentialCommandGroup(
                new InstantCommand(() -> m_drivetrainSubsystem.forceHeading(Rotation2d.fromDegrees(180))),
                new ZeroTelescope(m_telescopeSubsystem),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.HIGH_SCORE_FRONT),
                new FeederCommand(m_feederSubsystem, () -> -1).withTimeout(0.4),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.ZERO_POS),
                new FullAutoCommand(m_drivetrainSubsystem, "DirtyCube", autoBuilder));

        Command dirty_no_turn = new SequentialCommandGroup(
                new ZeroTelescope(m_telescopeSubsystem),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        new PinkPositions("", PositionConstants.HIGH_SCORE_BACK.pivot, 0, new Rotation2d(0))),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.HIGH_SCORE_BACK),
                new FeederCommand(m_feederSubsystem, () -> -1).withTimeout(0.4),
                new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem,
                        PositionConstants.ZERO_POS),
                new FullAutoCommand(m_drivetrainSubsystem, "Dirty No Turn", autoBuilder));

        Command nullAuto = null;

        autoChooser.setDefaultOption("No Auto :(", nullAuto);
        autoChooser.addOption("Score Preload High", score_preload_high);
        autoChooser.addOption("Score Preload Mid", score_preload_mid);
        autoChooser.addOption("Cube Mobility", cube_mobility);
        autoChooser.addOption("Cone Cube 2", cone_cube_2);
        autoChooser.addOption("Center One", center_one);
        autoChooser.addOption("Dirty Cube", dirty_cube);
        autoChooser.addOption("Dirty No Turn", dirty_no_turn);

        SmartDashboard.putData("Auto", autoChooser);
    }
}
