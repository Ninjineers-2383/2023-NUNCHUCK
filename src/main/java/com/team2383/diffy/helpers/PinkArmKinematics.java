package com.team2383.diffy.helpers;

import org.ejml.simple.SimpleMatrix;

import com.team2383.diffy.subsystems.TopPivotModule;

import edu.wpi.first.math.geometry.Rotation2d;

public class PinkArmKinematics {
    private SimpleMatrix forwardKinematics;
    private SimpleMatrix inverseKinematics;

    private double m_extensionInitLength;
    private double m_topPivotLength;

    private SimpleMatrix m_xValues;
    private SimpleMatrix m_yValues;

    public PinkArmKinematics(double extensionInitLength, double topPivotLength) {
        m_extensionInitLength = extensionInitLength;
        m_topPivotLength = topPivotLength;

        double[][] forwardKinematicInternals = {{1.0, topPivotLength / 2.0}};

        forwardKinematics = new SimpleMatrix(forwardKinematicInternals);

        inverseKinematics = forwardKinematics.pseudoInverse();
        
    }

    public PinkArmState toPinkArmState(Rotation2d bottomAngle, double extension, Rotation2d topAngle) {
        double[][] stateXMatrixInternals = {{Math.cos(bottomAngle.getRadians()) * extension}, {Math.cos(topAngle.getRadians()) * m_topPivotLength / 2.0}};
        SimpleMatrix stateXMatrix = new SimpleMatrix(stateXMatrixInternals);

        m_xValues = stateXMatrix.mult(inverseKinematics);

        double[][] stateYMatrixInternals = {{Math.sin(bottomAngle.getRadians()) * extension}, {Math.sin(topAngle.getRadians()) * m_topPivotLength / 2.0}};
        SimpleMatrix stateYMatrix = new SimpleMatrix(stateYMatrixInternals);

        m_yValues = stateYMatrix.mult(inverseKinematics);
    }


}
