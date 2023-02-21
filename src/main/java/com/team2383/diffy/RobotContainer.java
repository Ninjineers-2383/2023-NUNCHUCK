// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.diffy;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.diffy.autos.FullAutoCommand;
import com.team2383.diffy.commands.PaddleCommand;
import com.team2383.diffy.commands.FeederCommand;
import com.team2383.diffy.commands.JoystickDriveCommand;
import com.team2383.diffy.commands.pinkArm.PinkArmPresetCommand;
import com.team2383.diffy.subsystems.drivetrain.DrivetrainSubsystem;
import com.team2383.diffy.subsystems.paddle.PaddleSubsystem;
import com.team2383.diffy.subsystems.pinkArm.PinkArmSimSubsystem;
import com.team2383.diffy.subsystems.pinkArm.feeder.FeederSubsystem;
import com.team2383.diffy.subsystems.pinkArm.pivot.PivotSubsystem;
import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;
import com.team2383.diffy.subsystems.pinkArm.wrist.WristSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
    private final DoubleSupplier m_driveY = () -> MathUtil
            .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveX), .1);

    private final DoubleSupplier m_driveX = () -> MathUtil
            .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveY), .1);

    private final DoubleSupplier m_driveOmega = () -> MathUtil
            .applyDeadband(m_driverController.getRawAxis(Constants.OI.DriveOmega) * 0.5, .1);

    private final BooleanSupplier m_fieldCentric = () -> !(m_driverController.getRawButton(1));

    private final IntSupplier m_povSupplier = () -> -1;

    private final DoubleSupplier m_intake = () -> MathUtil
            .applyDeadband(m_driverController.getRawAxis(3) - m_driverController.getRawAxis(4), .1);

//     private final DoubleSupplier m_extension = () -> MathUtil
//             .applyDeadband(m_operatorController.getRawAxis(1) - m_operatorController.getRightY
//             (), .1);

    private final JoystickButton m_presetFeed = new JoystickButton(m_operatorController, 1);
    private final JoystickButton m_presetShootLow = new JoystickButton(m_operatorController, 2);
    private final JoystickButton m_presetShootHigh = new JoystickButton(m_operatorController, 3);

    private final DoubleSupplier m_dickControl = () -> 0.3 *
    (m_operatorController.getLeftTriggerAxis()
    - m_operatorController.getRightTriggerAxis());

    // The robot's subsystems and commands are defined here...
    
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(DataLogManager.getLog());
    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(DataLogManager.getLog());
    private final PaddleSubsystem m_dickSubsystem = new PaddleSubsystem();
    private Supplier<Rotation2d> m_pivotSupplier;
    private final TelescopeSubsystem m_telescopeSubsystem = new TelescopeSubsystem(m_pivotSupplier);
    private final WristSubsystem m_wristSubsystem = new WristSubsystem(m_pivotSupplier);
    private final PivotSubsystem m_pivotSubsystem = new PivotSubsystem(m_telescopeSubsystem::getExtensionInches);

    
    private final PinkArmSimSubsystem m_pinkArmSimSubsystem = new PinkArmSimSubsystem(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem);

    // Commands are defined here

    private final JoystickDriveCommand m_driveCommand = new JoystickDriveCommand(m_drivetrainSubsystem, m_driveX,
            m_driveY, m_driveOmega, m_fieldCentric, m_povSupplier);
    private final FeederCommand m_feederCommand = new FeederCommand(m_feederSubsystem, m_intake);
    private final PaddleCommand m_dickCommand = new PaddleCommand(m_dickSubsystem,
    m_dickControl);

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>() {
        {
            put("log", new PrintCommand("Event: log"));
            put("intakeDown", null);
        }
    };

    SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
            m_drivetrainSubsystem::getPose,
            m_drivetrainSubsystem::forceOdometry,
            m_drivetrainSubsystem.m_kinematics,
            new PIDConstants(5, 0, 0),
            new PIDConstants(0.5, 0, 0),
            m_drivetrainSubsystem::setModuleStates,
            eventMap,
            true,
            m_drivetrainSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_pivotSupplier = m_pivotSubsystem::getAngle;
        // Configure the button bindings
        configureButtonBindings();
        configureDefaultCommands();

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        LiveWindow.enableAllTelemetry();

        setAutoCommands();
    }

    private void configureButtonBindings() {
        m_presetFeed.onTrue(new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem, 
                Rotation2d.fromDegrees(150), -1, Rotation2d.fromDegrees(120)));

        m_presetShootLow.onTrue(new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem, 
                Rotation2d.fromDegrees(-60), 2, Rotation2d.fromDegrees(-90)));

        m_presetShootHigh.onTrue(new PinkArmPresetCommand(m_pivotSubsystem, m_telescopeSubsystem, m_wristSubsystem, 
                Rotation2d.fromDegrees(90), 1, Rotation2d.fromDegrees(60)));
    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);
        m_feederSubsystem.setDefaultCommand(m_feederCommand);
        m_dickSubsystem.setDefaultCommand(m_dickCommand);
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
        return null;//m_pinkArmAutoCommand;
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
