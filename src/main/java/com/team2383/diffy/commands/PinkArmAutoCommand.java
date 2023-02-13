package com.team2383.diffy.commands;

import java.util.function.DoubleSupplier;

import com.team2383.diffy.subsystems.PinkArm.PinkArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmAutoCommand extends CommandBase {
    private final double x;
    private final double y;
    private final double topAngle;
    private final PinkArmSubsystem pinkArm;

    public PinkArmAutoCommand(PinkArmSubsystem pinkArm, double x, double y, double topAngle) {
        this.x = x;
        this.y = y;
        this.topAngle = topAngle;
        this.pinkArm = pinkArm;

        addRequirements(pinkArm);
    }

    @Override
    public void initialize() {
        pinkArm.setPosition(x, y, topAngle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
