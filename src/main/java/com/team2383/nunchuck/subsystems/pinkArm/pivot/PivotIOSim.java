package com.team2383.nunchuck.subsystems.pinkArm.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class PivotIOSim implements PivotIO {
    private final FlywheelSim m_motor;

    private double volts;

    public PivotIOSim() {
        m_motor = new FlywheelSim(PivotConstants.SIMULATION_SUBSYSTEM, DCMotor.getNEO(2), 1);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        m_motor.update(0.02);

        inputs.angle += (m_motor.getAngularVelocityRPM() / 60) * 0.02;
        inputs.velocity = m_motor.getAngularVelocityRPM() / 60;
        inputs.appliedVolts = volts;
        inputs.currentLeft = m_motor.getCurrentDrawAmps();
        inputs.currentRight = m_motor.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double voltage) {
        volts = voltage;
        m_motor.setInputVoltage(voltage);
    }
}
