package com.team2383.diffy.commands.pinkArm.zero;

import com.team2383.diffy.subsystems.pinkArm.telescope.TelescopeSubsystem;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ZeroTelescope extends CommandBase {

    private TelescopeSubsystem m_telescope;

    private int count = 0;

    public ZeroTelescope(TelescopeSubsystem telescopeSubsystem) {
        m_telescope = telescopeSubsystem;

        addRequirements(m_telescope);
    }

    @Override
    public void initialize() {
        m_telescope.forceGoal(-10);
    }

    @Override
    public void end(boolean interrupted) {
        m_telescope.resetPosition();
        m_telescope.setGoal(0);
        DataLogManager.log("Zeero Telescope ");
    }

    @Override
    public boolean isFinished() {
        if (m_telescope.getCurrent() > 17) {
            count++;
        } else {
            count = 0;
        }
        return count > 5;
    }
}
