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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import com.team2383.diffy.autos.FullAutoCommand;
import com.team2383.diffy.commands.JoystickDriveCommand;
import com.team2383.diffy.commands.PinkArmTeleCommand;
import com.team2383.diffy.subsystems.DrivetrainSubsystem;
import com.team2383.diffy.subsystems.PinkArmSubsystem;

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
    private final Joystick m_driverMoveController = new Joystick(0);
    private final Joystick m_driverTurnController = new Joystick(1);
    private final XboxController m_operatorController = new XboxController(2);

    // Power and suppliers are defined here
    private final DoubleSupplier m_driveX = () -> m_driverMoveController.getX();
    private final DoubleSupplier m_driveY = () -> m_driverMoveController.getY();
    private final DoubleSupplier m_driveOmega = () -> m_driverTurnController.getX();
    private final BooleanSupplier m_fieldCentric = () -> !(m_driverMoveController.getTrigger()
            || m_driverTurnController.getTrigger());
    private final IntSupplier m_povSupplier = () -> m_driverTurnController.getPOV();

    private final IntSupplier m_bottomArm = m_operatorController.getAButton() ? () -> 1
            : m_operatorController.getYButton() ? () -> -1 : () -> 0;
    private final IntSupplier m_topArm = m_operatorController.getBButton() ? () -> 1
            : m_operatorController.getXButton() ? () -> -1 : () -> 0;
    private final IntSupplier m_extend = m_operatorController.getLeftBumper() ? () -> 1
            : m_operatorController.getRightBumper() ? () -> -1 : () -> 0;

    // The robot's subsystems and commands are defined here...
    private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(DataLogManager.getLog());
    private final PinkArmSubsystem m_pinkArmSubsystem = new PinkArmSubsystem(DataLogManager.getLog());

    private final JoystickDriveCommand m_dDriveCommand = new JoystickDriveCommand(m_drivetrainSubsystem, m_driveX,
            m_driveY, m_driveOmega, m_fieldCentric, m_povSupplier);
    private final PinkArmTeleCommand m_pinkArmCommand = new PinkArmTeleCommand(m_pinkArmSubsystem, m_bottomArm,
            m_topArm, m_extend);

    // This is just an example event map. It would be better to have a constant,
    // global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>() {
        {
            put("log", new PrintCommand("Event: log"));
            put("intakeDown", null);
        }
    };

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

    }

    private void configureButtonBindings() {
    }

    private void configureDefaultCommands() {
        m_drivetrainSubsystem.setDefaultCommand(m_dDriveCommand);
        m_pinkArmSubsystem.setDefaultCommand(m_pinkArmCommand);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new FullAutoCommand(m_drivetrainSubsystem, "Forward", eventMap);
    }
}
