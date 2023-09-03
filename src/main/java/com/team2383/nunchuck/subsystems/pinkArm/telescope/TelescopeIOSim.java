package com.team2383.nunchuck.subsystems.pinkArm.telescope;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class TelescopeIOSim implements TelescopeIO {
    private final FlywheelSim m_motor;

    private double volts;
    private double extension = 0;

    public TelescopeIOSim() {
        m_motor = new FlywheelSim(TelescopeConstants.SIMULATION_SUBSYSTEM, DCMotor.getNEO(2), 1);
    }

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        m_motor.update(0.02);

        extension += (m_motor.getAngularVelocityRPM() / 60) * 0.02 * TelescopeConstants.ROTATION_CONVERSION;

        inputs.extension = extension;
        inputs.velocity = m_motor.getAngularVelocityRPM() * TelescopeConstants.ROTATION_CONVERSION / 60;
        inputs.appliedVolts = volts;
        inputs.current = m_motor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        volts = voltage;
        m_motor.setInputVoltage(voltage);
    }

    @Override
    public void setEncoderPosition(double position) {
        extension = position;
    }
}
