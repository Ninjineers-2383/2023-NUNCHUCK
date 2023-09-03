package com.team2383.nunchuck.subsystems.pinkArm.wrist;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class WristIOSim implements WristIO {
    private final FlywheelSim m_motor;

    private double volts;

    public WristIOSim() {
        m_motor = new FlywheelSim(WristConstants.SIMULATION_SUBSYSTEM, DCMotor.getNEO(2), 1);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        m_motor.update(0.02);

        inputs.angle -= (m_motor.getAngularVelocityRPM() / 60) * 0.02;
        inputs.velocity = m_motor.getAngularVelocityRPM() / 60;
        inputs.appliedVolts = volts;
        inputs.current = m_motor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        volts = voltage;
        m_motor.setInputVoltage(voltage);
    }
}
