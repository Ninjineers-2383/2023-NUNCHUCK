package com.team2383.nunchuck.subsystems.pinkArm.feeder;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final FeederIO m_io;
    private final FeederIOInputsAutoLogged m_inputs = new FeederIOInputsAutoLogged();

    public FeederSubsystem(FeederIO io) {
        m_io = io;
    }

    @Override
    public void periodic() {
        m_io.updateInputs(m_inputs);

        Logger.getInstance().processInputs("Feeder", m_inputs);
    }

    public void setPower(double power) {
        m_io.setPower(power);
    }
}
