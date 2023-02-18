package com.team2383.diffy.commands.pinkArm;

import com.team2383.diffy.subsystems.pinkArm.PinkArmSimSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class PinkArmSimCommand extends CommandBase {
    private final PinkArmSimSubsystem m_pinkArmSim;
    
    public PinkArmSimCommand(PinkArmSimSubsystem pinkArmSim) {
        m_pinkArmSim = pinkArmSim;

        addRequirements(m_pinkArmSim);
    }

    @Override
    public void execute() {

    }
}
