package com.team2383.diffy.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import com.team2383.diffy.Constants.IntakeConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    //TODO: Comment
    private final CANSparkMax m_topMotor;
    private final CANSparkMax m_bottomMotor;
    private final CANSparkMax m_clawMotor;

    private final 

    private final LinearSystem<N3, N3, N3> m_feederPlant;
    private final LinearQuadraticRegulator<N3, N3, N3> m_controller;
    private final KalmanFilter<N3, N3, N3> m_observer;
    private final LinearSystemLoop<N3, N3, N3> m_systemLoop;


    public IntakeSubsystem() {
        m_topMotor = new CANSparkMax(IntakeConstants.kTopMotorID, MotorType.kBrushless);
        m_bottomMotor = new CANSparkMax(IntakeConstants.kBottomMotorID, MotorType.kBrushless);
        m_clawMotor = new CANSparkMax(IntakeConstants.kClawMotorID, MotorType.kBrushless);

        // RevPhysicsSim.getInstance().addSparkMax(m_topMotor, MotorType.kBrushless);

        m_feederPlant = new LinearSystem<>(
            // A Matrix
            Matrix.mat(Nat.N3(), Nat.N3()).fill(
                -IntakeConstants.kV / IntakeConstants.kA, 0, 0,
                0, -IntakeConstants.kV / IntakeConstants.kA, 0,
                0, 0, -IntakeConstants.kV / IntakeConstants.kA),

            // B Matrix
            Matrix.mat(Nat.N3(), Nat.N3()).fill(
                1 / IntakeConstants.kA, 0, 0,
                0, 1 / IntakeConstants.kA, 0,
                0, 0, 1 / IntakeConstants.kA),

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
            VecBuilder.fill(1, 1, 1), VecBuilder.fill(12, 12, 12), 0.02);

        m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), m_feederPlant, 
            VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.1, 0.1, 0.1), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_feederPlant, m_controller, 
            m_observer, 12.0, 0.02);
    }

    public void periodic() {
     
    }

    public void simulate() {
        // m_leftMotorSim.setIntegratedSensorVelocity((int) ((m_systemLoop.getXHat(0) / (2 * Math.PI)) * 2048 / 10.0));
        // m_rightMotorSim.setIntegratedSensorVelocity((int) ((m_systemLoop.getXHat(1) / (2 * Math.PI)) * 2048 / 10.0));

        // SmartDashboard.putNumber("Simulated Left Motor Output Velocity",
        //         m_leftMotor.getSelectedSensorVelocity());

        // SmartDashboard.putNumber("Simulated Right Motor Output Velocity",
        //         m_rightMotor.getSelectedSensorVelocity());

    }

    public void setPower(double top, double bottom) {
        m_topMotor.set(top);
        m_bottomMotor.set(bottom);
    }

 
}
