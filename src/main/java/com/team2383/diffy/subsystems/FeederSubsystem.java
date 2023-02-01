package com.team2383.diffy.subsystems;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Constants.FeederConstants;
import com.team2383.diffy.helpers.Ninja_CANSparkMax;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSubsystem extends SubsystemBase {
    //TODO: Comment
    private final Ninja_CANSparkMax m_topMotor;
    private final Ninja_CANSparkMax m_bottomMotor;

    // SS Controller
    private final LinearSystem<N2, N2, N2> m_feederPlant;
    private final LinearQuadraticRegulator<N2, N2, N2> m_controller;
    private final KalmanFilter<N2, N2, N2> m_observer;
    private final LinearSystemLoop<N2, N2, N2> m_systemLoop;

    private double m_topSpeed;
    private double m_bottomSpeed;

    private double m_desiredTopSpeed;
    private double m_desiredBottomSpeed;

    private double m_bottomVoltage;
    private double m_topVoltage;

    private DataLog m_log;

    private final DoubleLogEntry m_topMotorCurrent;
    private final DoubleLogEntry m_bottomMotorCurrent;

    private final DoubleLogEntry m_topMotorVel;
    private final DoubleLogEntry m_bottomMotorVel;

    private final DoubleLogEntry m_expectedTopSpeed;
    private final DoubleLogEntry m_expectedBottomSpeed;

    public FeederSubsystem(DataLog log) {
        // Declare motor instances
        m_topMotor = new Ninja_CANSparkMax(Constants.FeederConstants.kTopMotorID, MotorType.kBrushless);
        m_bottomMotor = new Ninja_CANSparkMax(Constants.FeederConstants.kBottomMotorID, MotorType.kBrushless);
       
        m_feederPlant = new LinearSystem<>(
            // A Matrix
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                -FeederConstants.kV / FeederConstants.kA, 0,
                0, -FeederConstants.kV / FeederConstants.kA),

            // B Matrix
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                1 / FeederConstants.kA, 0,
                0, 1 / FeederConstants.kA),

            // C Matrix
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                1, 0,
                0, 1), 

            // D Matrix
            Matrix.mat(Nat.N2(), Nat.N2()).fill(
                0, 0,
                0, 0)
        );
        
        m_controller = new LinearQuadraticRegulator<>(m_feederPlant, 
            VecBuilder.fill(1, 1), VecBuilder.fill(12, 12), 0.02);

        m_observer = new KalmanFilter<>(Nat.N2(), Nat.N2(), m_feederPlant, 
            VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1, 0.1), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_feederPlant, m_controller, 
            m_observer, 12.0, 0.02);

        m_log = log;

        m_topMotorCurrent = new DoubleLogEntry(m_log, "/topMotorCurrent");
        m_bottomMotorCurrent = new DoubleLogEntry(m_log, "/bottomMotorCurrent");


        m_topMotorVel = new DoubleLogEntry(m_log, "/topMotorVel");
        m_bottomMotorVel = new DoubleLogEntry(m_log, "/bottomMotorVel");

        m_expectedTopSpeed = new DoubleLogEntry(m_log, "/topExpectedVel");
        m_expectedBottomSpeed = new DoubleLogEntry(m_log, "/bottomExpectedVel");

        addChild("Feeder", this);
    }

    @Override
    public void periodic() {
        m_topSpeed = m_topMotor.get();
        m_bottomSpeed = m_bottomMotor.get();

        m_topMotorCurrent.append(m_topMotor.getOutputCurrent());
        m_bottomMotorCurrent.append(m_bottomMotor.getOutputCurrent());

        m_topMotorVel.append(m_topSpeed);
        m_bottomMotorVel.append(m_bottomSpeed);

        m_expectedTopSpeed.append(m_desiredTopSpeed);
        m_expectedBottomSpeed.append(m_desiredBottomSpeed);
    }

    @Override
    public void simulationPeriodic() {
        // Set simulated VictorSPX voltage
        m_bottomMotor.set(m_systemLoop.getXHat(0));
        m_topMotor.set(m_systemLoop.getXHat(1));

        SmartDashboard.putNumber("Simulated Top Motor Feeder Big Booty Bitches Output Velocity",
                m_topMotor.get());

        SmartDashboard.putNumber("Simulated Bottom Motor Feeder Output Velocity",
                m_bottomMotor.get());
    }

    public void setPower(double top, double bottom) {
        // Set discrete motor power
        m_desiredTopSpeed = top;
        m_desiredBottomSpeed = bottom;

        m_systemLoop.setNextR(VecBuilder.fill(m_desiredBottomSpeed, m_desiredTopSpeed));

        m_systemLoop.correct(VecBuilder.fill(m_bottomSpeed, m_topSpeed));

        m_systemLoop.predict(0.02);

        m_bottomVoltage = m_systemLoop.getU(0);
        m_topVoltage = m_systemLoop.getU(1);

        setVoltage();
    }

    public void setVoltage() {
        m_bottomMotor.setVoltage(m_bottomVoltage);
        m_topMotor.setVoltage(m_topVoltage);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Telescope");

        builder.addDoubleProperty("Desired Bottom Speed", () -> {
            return m_desiredBottomSpeed;
        }, null);

        builder.addDoubleProperty("Desired Top Speed", () -> {
            return m_desiredTopSpeed;
        }, null);

        builder.addDoubleProperty("Top Speed", () -> {
            return m_topSpeed;
        }, null);

        builder.addDoubleProperty("Bottom Speed", () -> {
            return m_bottomSpeed;
        }, null);

        builder.addDoubleProperty("Top Voltage", () -> {
            return m_topVoltage;
        }, null);
        
        builder.addDoubleProperty("Bottom Voltage", () -> {
            return m_bottomVoltage;
        }, null);

        builder.addDoubleProperty("Estimated Bottom Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(0);
        }, null);

        builder.addDoubleProperty("Estimated Top Velocity (x hat)", () -> {
            return m_systemLoop.getXHat(1);
        }, null);
    }

}
