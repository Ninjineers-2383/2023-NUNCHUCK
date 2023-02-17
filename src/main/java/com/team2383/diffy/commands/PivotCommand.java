// TODO: Implement Command
package com.team2383.diffy.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.diffy.subsystems.pinkArm.pivot.*;

public class PivotCommand extends CommandBase {

    private final PivotSubsystem pivot;
    private final double velocity;

    public PivotCommand(PivotSubsystem pivot, double velocity) {
        this.pivot = pivot;
        this.velocity = velocity;
        addRequirements(pivot);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        // Keeping extension at 0, seems sketchy to rotate and extend at the same time
        pivot.setAngle(velocity, 0);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
