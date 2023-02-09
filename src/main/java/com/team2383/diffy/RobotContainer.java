// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.diffy;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.team2383.diffy.autos.FullAutoCommand;
import com.team2383.diffy.commands.FeederCommand;
import com.team2383.diffy.commands.JoystickDriveCommand;
import com.team2383.diffy.commands.PinkArmTeleCommand;
import com.team2383.diffy.commands.PinkArmTestCommand;
import com.team2383.diffy.commands.DickCommand;

import com.team2383.diffy.subsystems.DrivetrainSubsystem;
import com.team2383.diffy.subsystems.FeederSubsystem;
import com.team2383.diffy.subsystems.PinkArmSubsystem;
import com.team2383.diffy.subsystems.DickSubsystem;

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
    private final DoubleSupplier m_driveY = () -> m_driverController.getRawAxis(0) / 0.6;
    private final DoubleSupplier m_driveX = () -> m_driverController.getRawAxis(1) / 0.6;
    private final DoubleSupplier m_driveOmega = () -> m_driverController.getRawAxis(2) / 0.6;
    private final BooleanSupplier m_fieldCentric = () -> !(m_driverController.getRawButton(1));
    private final IntSupplier m_povSupplier = () -> -1;

    private final BooleanSupplier m_bottomArmUp = () -> m_operatorController.getAButton();
    private final BooleanSupplier m_bottomArmDown = () -> m_operatorController.getYButton();
    private final BooleanSupplier m_topArmUp = () -> m_operatorController.getXButton();
    private final BooleanSupplier m_topArmDown = () -> m_operatorController.getBButton();
    private final BooleanSupplier m_extensionUp = () -> m_operatorController.getLeftBumper();
    private final BooleanSupplier m_extensionDown = () -> m_operatorController.getRightBumper();

    private final BooleanSupplier m_intake = () -> m_operatorController.getLeftBumper();
    private final BooleanSupplier m_outtake = () -> m_operatorController.getRightBumper();

    private final DoubleSupplier m_dickControl = () -> 0.3 * (m_operatorController.getLeftTriggerAxis()
            - m_operatorController.getRightTriggerAxis());

    // The robot's subsystems and commands are defined here...
    // private final DrivetrainSubsystem m_drivetrainSubsystem = new
    // DrivetrainSubsystem(DataLogManager.getLog());
    private final PinkArmSubsystem m_pinkArmSubsystem = new PinkArmSubsystem(DataLogManager.getLog());
    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem(DataLogManager.getLog());
    private final DickSubsystem m_dickSubsystem = new DickSubsystem();

    // private final JoystickDriveCommand m_driveCommand = new
    // JoystickDriveCommand(m_drivetrainSubsystem, m_driveX,
    // m_driveY,m_driveOmega,m_fieldCentric,m_povSupplier);

    private final PinkArmTestCommand m_pinkArmCommand = new PinkArmTestCommand(m_pinkArmSubsystem, m_bottomArmDown,
            m_bottomArmUp, m_topArmDown, m_topArmUp, m_extensionDown, m_extensionUp);

    private final FeederCommand m_feederCommand = new FeederCommand(m_feederSubsystem, m_intake, m_outtake);

    private final DickCommand m_dickCommand = new DickCommand(m_dickSubsystem, m_dickControl);

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

    // SwerveAutoBuilder m_autoBuilder = new SwerveAutoBuilder(
    // m_drivetrainSubsystem::getPose,
    // m_drivetrainSubsystem::forceOdometry,
    // m_drivetrainSubsystem.m_kinematics,
    // new PIDConstants(5, 0, 0),
    // new PIDConstants(0.5, 0, 0),
    // m_drivetrainSubsystem::setModuleStates,
    // eventMap,
    // true,
    // m_drivetrainSubsystem);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
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
    }

    private void configureDefaultCommands() {
        // m_drivetrainSubsystem.setDefaultCommand(m_driveCommand);
        m_pinkArmSubsystem.setDefaultCommand(m_pinkArmCommand);
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
        return autoChooser.getSelected();
    }

    private void setAutoCommands() {
        // Command forwardTest = new FullAutoCommand(m_drivetrainSubsystem, "Forward",
        // m_autoBuilder);
        // Command twoConeAuto = new FullAutoCommand(m_drivetrainSubsystem, "TwoCones",
        // m_autoBuilder);

        Command nullAuto = null;

        // autoChooser.setDefaultOption("Two Cone Path Auto", twoConeAuto);
        // autoChooser.addOption("Forward Test Auto", forwardTest);
        autoChooser.addOption("No Auto :(", nullAuto);
    }
}
