package com.team2383.nunchuck.subsystems.pinkArm.feeder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FeederIOSim implements FeederIO {
    private final FlywheelSim m_motor;
    private double volts;

    public FeederIOSim() {
        m_motor = new FlywheelSim(DCMotor.getFalcon500(1), 1, 0.0004);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        m_motor.update(0.02);

        inputs.velocityRPM = m_motor.getAngularVelocityRPM() / 60.0;
        inputs.appliedVolts = volts;
        inputs.current = m_motor.getCurrentDrawAmps();
    }

    @Override
    public void setPower(double dutyCycle) {
        m_motor.setInputVoltage(dutyCycle * 12.0);
    }

    @Override
    public void stop() {
        m_motor.setInputVoltage(0.0);
    }
}
