package com.team2383.diffy.subsystems;

import com.team2383.diffy.Constants.FeederConstants;

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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSubsystem extends SubsystemBase {
    //TODO: Comment
    private final CANSparkMax m_topMotor;
    private final CANSparkMax m_bottomMotor;
    private final CANSparkMax m_clawMotor;

    private final LinearSystem<N3, N3, N3> m_feederPlant;
    private final LinearQuadraticRegulator<N2, N1, N2> m_controller;
    private final KalmanFilter<N2, N1, N2> m_observer;
    private final LinearSystemLoop<N2, N1, N2> m_systemLoop;


    public FeederSubsystem() {
        m_topMotor = new CANSparkMax(Constants.FeederConstants.kTopMotorID, MotorType.kBrushless);
        m_bottomMotor = new CANSparkMax(Constants.FeederConstants.kBottomMotorID, MotorType.kBrushless);
        m_clawMotor = new CANSparkMax(Constants.FeederConstants.kClawMotorID, MotorType.kBrushless);

        m_feederPlant = new LinearSystem<N3, N3, N3>(
            // A Matrix
            Matrix.mat(Nat.N3(), Nat.N3()).fill(
                -FeederConstants.kV / FeederConstants.kA, 0, 0,
                0, -FeederConstants.kV / FeederConstants.kA, 0,
                0, 0, -FeederConstants.kV / FeederConstants.kA),

            // B Matrix
            Matrix.mat(Nat.N3(), Nat.N3()).fill(
                1 / FeederConstants.kA, 0, 0,
                0, 1 / FeederConstants.kA, 0,
                0, 0, 1 / FeederConstants.kA),

            // C Matrix
            Matrix.mat(Nat.N3(), Nat.N3()).fill(
                1, 0, 0,
                0, 1, 0,
                0, 0, 1), 

            // D Matrix
            Matrix.mat(Nat.N3(), Nat.N3()).fill(
                0, 0, 0,
                0, 0, 0,
                0, 0, 0)
        );
        
        m_controller = new LinearQuadraticRegulator<>(m_feederPlant, 
            VecBuilder.fill(1, 1), VecBuilder.fill(12), 0.02);

        m_observer = new KalmanFilter<>(Nat.N2(), Nat.N2(), m_feederPlant, 
            VecBuilder.fill(0.1, 0.1), VecBuilder.fill(0.1, 0.1), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_feederPlant, m_controller, 
            m_observer, 12.0, 0.02);


    public void periodic() {
     
    }

    public void setPower(double top, double bottom) {
        m_topMotor.set(top);
        m_bottomMotor.set(bottom);
    }

 
}
