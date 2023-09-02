package com.team2383.nunchuck.subsystems.pinkArm.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class WristIOTalonSRX implements WristIO {
    private final TalonSRX m_wristMotor;

    public WristIOTalonSRX() {
        m_wristMotor = new TalonSRX(WristConstants.kMotorID);
        m_wristMotor.configFactoryDefault();
        // Set postional offset
        m_wristMotor.configPeakCurrentLimit(WristConstants.kMaxCurrent, 500);
        m_wristMotor.enableCurrentLimit(true);
        m_wristMotor.configVoltageCompSaturation(12);
        m_wristMotor.enableVoltageCompensation(true);
        m_wristMotor.configVoltageMeasurementFilter(1);
        m_wristMotor.getSensorCollection()
                .setQuadraturePosition(m_wristMotor.getSensorCollection().getPulseWidthPosition(), 200);

        m_wristMotor.setInverted(true);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.angle = (m_wristMotor.getSensorCollection().getQuadraturePosition()
                - WristConstants.encoderOffset) / 4096.0;
        inputs.velocity = m_wristMotor.getSensorCollection().getQuadratureVelocity() / 4096.0 * 10;
        inputs.appliedVolts = m_wristMotor.getMotorOutputVoltage();
        inputs.current = m_wristMotor.getStatorCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        m_wristMotor.set(ControlMode.PercentOutput, voltage / 12);
    }
}
