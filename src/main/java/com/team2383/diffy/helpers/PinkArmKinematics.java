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

    public PinkArmState toPinkArmState(double x, double y) {
        double[][] stateXMatrixInternals = {{x}};
        SimpleMatrix stateXMatrix = new SimpleMatrix(stateXMatrixInternals);

        m_xValues = inverseKinematics.mult(stateXMatrix);

        double[][] stateYMatrixInternals = {{y}};
        SimpleMatrix stateYMatrix = new SimpleMatrix(stateYMatrixInternals);

        m_yValues = inverseKinematics.mult(stateYMatrix);

        double extension = Math.sqrt(x + y);

        double topAngle = Math.acos(m_xValues.get(1, 0));

        double bottomAngle = Math.acos(m_xValues.get(0, 0) / extension);

        return new PinkArmState(extension, new Rotation2d(bottomAngle), new Rotation2d(topAngle));
    }


}
