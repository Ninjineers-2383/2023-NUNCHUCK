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

import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import com.team2383.diffy.autos.*;
import com.team2383.diffy.commands.JoystickDriveCommand;
import com.team2383.diffy.subsystems.DrivetrainSubsystem;
import com.team2383.diffy.subsystems.DickSubsystem;
import com.team2383.diffy.commands.DickCommand;

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
    // The controls are defined here
    private final GenericHID m_driverMoveController = new GenericHID(0);
    private final XboxController m_operatorController = new XboxController(1);
    // private final Joystick m_driverTurnController = new Joystick(1);
    // private final XboxController m_operatorController = new XboxController(2);

    // Power and suppliers are defined here
    private final DoubleSupplier m_driveX = () -> m_driverMoveController.getRawAxis(1)/0.6;
    private final DoubleSupplier m_driveY = () -> m_driverMoveController.getRawAxis(0)/0.6;
    private final DoubleSupplier m_driveOmega = () -> m_driverMoveController.getRawAxis(3)/0.6;
    private final BooleanSupplier m_fieldCentric = () -> !(m_driverMoveController.getRawButton(1));
    private final IntSupplier m_povSupplier = () -> -1;

    private final DoubleSupplier m_erect = () -> m_operatorController.getRightTriggerAxis();
    private final DoubleSupplier m_flaccid = () -> m_operatorController.getLeftTriggerAxis();

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(DataLogManager.getLog());
    private final DickSubsystem m_dickSubsystem = new DickSubsystem();

    private final JoystickDriveCommand m_defaultDriveCommand = new JoystickDriveCommand(m_drivetrainSubsystem, m_driveX,
            m_driveY, m_driveOmega, m_fieldCentric, m_povSupplier);

    private final DickCommand m_defaultDickCommand = new DickCommand(m_dickSubsystem, () -> 
    (m_erect.getAsDouble() - m_flaccid.getAsDouble()) * 0.3);

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
        // Configure the button bindings
        configureButtonBindings();
        configureDefaultCommands();

        DataLogManager.start();
        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        LiveWindow.enableAllTelemetry();
    }

    private void configureButtonBindings() {
    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(m_defaultDriveCommand);
        m_dickSubsystem.setDefaultCommand(m_defaultDickCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // return new FullAutoCommand(m_drivetrainSubsystem, "Forward", m_autoBuilder);
        return new TwoConeAuto(m_drivetrainSubsystem, "TwoCones", m_autoBuilder);
    }
}
