package com.team2383.nunchuck.subsystems.pinkArm.feeder;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class FeederIOFalcon500 implements FeederIO {
    private final TalonFX m_motor;

    public FeederIOFalcon500() {
        m_motor = new TalonFX(FeederConstants.kMotorID);
        
        m_motor.configFactoryDefault();

        m_motor.setInverted(false);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.velocityRPM = m_motor.getSelectedSensorVelocity();
        inputs.appliedVolts = m_motor.getMotorOutputVoltage();
        inputs.current = m_motor.getSupplyCurrent();
    }

    @Override
    public void setPower(double dutyCycle) {
        m_motor.set(TalonFXControlMode.PercentOutput, dutyCycle);
    }

    @Override
    public void stop() {
        m_motor.set(TalonFXControlMode.PercentOutput, 0.0);
    }
}
