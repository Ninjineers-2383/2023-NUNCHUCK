// TODO: Implement Command
package com.team2383.diffy.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.diffy.subsystems.pinkArm.wrist.*;

public class WristCommand extends CommandBase {

    private final WristSubsystem wrist;
    private final double velocity;

    public WristCommand(WristSubsystem wrist, double velocity) {
        this.wrist = wrist;
        this.velocity = velocity;
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        wrist.setVelocity(velocity);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}