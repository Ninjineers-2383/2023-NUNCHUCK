package com.team2383.nunchuck.subsystems.pinkArm.telescope;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class TelescopeIOSparkMax implements TelescopeIO {
    private final CANSparkMax m_motor;

    public TelescopeIOSparkMax() {
        m_motor = new CANSparkMax(TelescopeConstants.EXTENSION_ID, MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setSmartCurrentLimit(TelescopeConstants.MAX_CURRENT);
        m_motor.setInverted(true);
    }

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        inputs.extension = m_motor.getEncoder().getPosition() * TelescopeConstants.ROTATION_CONVERSION;
        inputs.velocity = m_motor.getEncoder().getVelocity() * TelescopeConstants.ROTATION_CONVERSION / 60.0;
        inputs.appliedVolts = m_motor.getAppliedOutput();
        inputs.current = m_motor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    @Override
    public void setEncoderPosition(double position) {
        m_motor.getEncoder().setPosition(0);
    }
}
