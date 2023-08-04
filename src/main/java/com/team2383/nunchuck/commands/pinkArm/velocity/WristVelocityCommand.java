package com.team2383.nunchuck.commands.pinkArm.velocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.Supplier;

import com.team2383.nunchuck.subsystems.pinkArm.wrist.*;

public class WristVelocityCommand extends CommandBase {

    private final WristSubsystem m_wrist;
    private final Supplier<Rotation2d> m_angle;

    public WristVelocityCommand(WristSubsystem wrist, Supplier<Rotation2d> desiredAngle) {
        this.m_wrist = wrist;
        this.m_angle = desiredAngle;
        addRequirements(m_wrist);
    }

    public void execute() {
        m_wrist.setVelocity(m_angle.get());
    }
}