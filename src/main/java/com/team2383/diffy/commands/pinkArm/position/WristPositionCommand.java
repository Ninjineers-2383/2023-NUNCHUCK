package com.team2383.diffy.commands.pinkArm.position;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.diffy.subsystems.pinkArm.wrist.*;

public class WristPositionCommand extends CommandBase {

    private final WristSubsystem m_wrist;
    private final Rotation2d m_angle;

    public WristPositionCommand(WristSubsystem wrist, Rotation2d desiredAngle) {
        this.m_wrist = wrist;
        this.m_angle = desiredAngle;
        addRequirements(m_wrist);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        m_wrist.setGoal(Rotation2d.fromRadians(m_angle));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return m_wrist.isAtPosition();
    }
}