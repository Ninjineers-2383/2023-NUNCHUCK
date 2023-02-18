package com.team2383.diffy.subsystems.pinkArm.feeder;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSubsystem extends SubsystemBase {
    private final Ninja_CANSparkMax m_topMotor;
    private final Ninja_CANSparkMax m_bottomMotor;

    private double m_bottomVoltage;
    private double m_topVoltage;

    private DataLog m_log;

    private final DoubleLogEntry m_topMotorCurrent;
    private final DoubleLogEntry m_bottomMotorCurrent;

    public FeederSubsystem(DataLog log) {
        // Declare motor instances
        m_topMotor = new Ninja_CANSparkMax(FeederConstants.kTopMotorID, MotorType.kBrushless);
        m_bottomMotor = new Ninja_CANSparkMax(FeederConstants.kBottomMotorID, MotorType.kBrushless);

        m_log = log;

        m_topMotorCurrent = new DoubleLogEntry(m_log, "/topMotorCurrent");
        m_bottomMotorCurrent = new DoubleLogEntry(m_log, "/bottomMotorCurrent");

        addChild("Feeder", this);
    }

    @Override
    public void periodic() {
        m_topMotorCurrent.append(m_topMotor.getOutputCurrent());
        m_bottomMotorCurrent.append(m_bottomMotor.getOutputCurrent());
    }

    @Override
    public void simulationPeriodic() {
        // Set simulated VictorSPX voltage

        SmartDashboard.putNumber("Simulated Top Motor Feeder Big Booty Bitches Output Velocity",
                m_topMotor.get());

        SmartDashboard.putNumber("Simulated Bottom Motor Feeder Output Velocity",
                m_bottomMotor.get());
    }

    public void setPower(double top, double bottom) {
        // Set discrete motor power

        m_bottomVoltage = bottom;
        m_topVoltage = top;

        setVoltage();
    }

    public void setVoltage() {
        m_bottomMotor.setVoltage(m_bottomVoltage);
        m_topMotor.setVoltage(m_topVoltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Telescope");

        builder.addDoubleProperty("Top Voltage", () -> {
            return m_topVoltage;
        }, null);

        builder.addDoubleProperty("Bottom Voltage", () -> {
            return m_bottomVoltage;
        }, null);
    }

}
