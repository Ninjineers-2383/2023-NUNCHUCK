package com.team2383.diffy.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXSimCollection;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team2383.diffy.Constants;
import com.team2383.diffy.Constants.FeederConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FeederSubsystem extends SubsystemBase {
    //TODO: Comment
    private final VictorSPX m_topMotor;
    private final VictorSPX m_bottomMotor;
    //private final VictorSPX m_clawMotor; 
    // Sim Support
    private final VictorSPXSimCollection m_topMotorSim;
    private final VictorSPXSimCollection m_bottomMotorSim;
    //private final VictorSPXSimCollection m_clawMotorSim;

    // SS Controller
    private final LinearSystem<N3, N3, N3> m_feederPlant;
    private final LinearQuadraticRegulator<N3, N3, N3> m_controller;
    private final KalmanFilter<N3, N3, N3> m_observer;
    private final LinearSystemLoop<N3, N3, N3> m_systemLoop;


    public FeederSubsystem() {
        // Declare motor instances
        m_topMotor = new VictorSPX(Constants.FeederConstants.kTopMotorID);
        m_bottomMotor = new VictorSPX(Constants.FeederConstants.kBottomMotorID);
        //m_clawMotor = new VictorSPX(Constants.FeederConstants.kClawMotorID);

        m_topMotorSim = m_topMotor.getSimCollection();
        m_bottomMotorSim = m_bottomMotor.getSimCollection();
        //m_clawMotorSim = m_clawMotor.getSimCollection();
       
        m_feederPlant = new LinearSystem<>(
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
            VecBuilder.fill(1, 1, 1), VecBuilder.fill(12, 12, 12), 0.02);

        m_observer = new KalmanFilter<>(Nat.N3(), Nat.N3(), m_feederPlant, 
            VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.1, 0.1, 0.1), 0.02);

        m_systemLoop = new LinearSystemLoop<>(m_feederPlant, m_controller, 
            m_observer, 12.0, 0.02);
    }

    public void periodic() {
     
    }

    public void simulate() {
        // Set simulated VictorSPX voltage
        m_topMotorSim.setBusVoltage((int) ((m_systemLoop.getXHat(0) / (2 * Math.PI)) * 2048 / 10.0));
        m_bottomMotorSim.setBusVoltage((int) ((m_systemLoop.getXHat(1) / (2 * Math.PI)) * 2048 / 10.0));

        SmartDashboard.putNumber("Simulated Top Motor Output Velocity",
                m_topMotor.getSelectedSensorVelocity());

        SmartDashboard.putNumber("Simulated Bottom Motor Output Velocity",
                m_bottomMotor.getSelectedSensorVelocity());
    }

    public void setPower(double top, double bottom) {
        // Set discrete motor power
        m_topMotor.set(ControlMode.PercentOutput, top);
        m_bottomMotor.set(ControlMode.PercentOutput, bottom);
    }
}
