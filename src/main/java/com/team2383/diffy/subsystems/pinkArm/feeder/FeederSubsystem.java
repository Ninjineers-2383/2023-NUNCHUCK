package com.team2383.diffy.subsystems.pinkArm.feeder;

import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final TalonFX m_feederMotor;

    private final VoltageOut m_bottomVoltage;

    private double m_motorPower;

    public FeederSubsystem(DataLog log) {
        // TODO: Add CANivore if it's being used
        m_feederMotor = new TalonFX(FeederConstants.kTopMotorID);

        m_bottomVoltage = new VoltageOut(0, false, false);

        m_feederMotor.getConfigurator().apply(FeederConstants.kMotorConfigs.feederConfig());

        addChild("Feeder", this);
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
    }

    /**
     * Sets the power of the feeder motor, 0 is off, and 1 and -1 are the max speed
     * in either direction
     * 
     * @param power from -1 to 1
     */
    public void setPower(double power) {
        // Set discrete motor power
        m_motorPower = power;

        setVoltage();
    }

    public void setVoltage() {
        m_feederMotor.setControl(m_bottomVoltage.withOutput(m_motorPower * 12));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Feeder?");

        builder.addDoubleProperty("Feeder Voltage", () -> {
            return m_bottomVoltage.Output;
        }, null);

    }

}
