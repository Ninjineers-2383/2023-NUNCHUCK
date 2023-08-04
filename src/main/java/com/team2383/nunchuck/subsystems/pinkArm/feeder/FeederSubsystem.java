package com.team2383.nunchuck.subsystems.pinkArm.feeder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX m_motor;

    private double m_output;

    private DataLog m_log;

    private final DoubleLogEntry m_motorCurrent;

    public FeederSubsystem(DataLog log) {
        // Declare motor instances
        m_motor = new TalonFX(FeederConstants.kMotorID);

        m_log = log;

        m_motorCurrent = new DoubleLogEntry(m_log, "/feeder/motorCurrent");

        m_motor.configFactoryDefault();

        m_motor.setInverted(false);
    }

    @Override
    public void periodic() {
        m_motorCurrent.append(m_motor.getSupplyCurrent());
    }

    @Override
    public void simulationPeriodic() {
    }

    public void setPower(double power) {
        m_output = power;

        setVoltage();
    }

    public void setVoltage() {
        m_motor.set(ControlMode.PercentOutput, m_output);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Telescope");

        builder.addDoubleProperty("Voltage", () -> {
            return m_output;
        }, null);
    }
}
