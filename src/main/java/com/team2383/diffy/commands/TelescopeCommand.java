// TODO: Implement Command

package com.team2383.diffy.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import com.team2383.diffy.subsystems.pinkArm.telescope.*;

public class TelescopeCommand extends CommandBase {

    private final TelescopeSubsystem telescope;
    private final double extension;

    public TelescopeCommand(TelescopeSubsystem telescope, double extension) {
        this.telescope = telescope;
        this.extension = extension;
        addRequirements(telescope);
    }

    @Override
    public void initialize() {
    }

    public void execute() {
        telescope.setExtension(extension);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}