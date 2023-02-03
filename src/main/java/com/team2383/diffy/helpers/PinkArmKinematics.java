package com.team2383.diffy.helpers;

import org.ejml.simple.SimpleMatrix;

import edu.wpi.first.math.geometry.Rotation2d;

public class PinkArmKinematics {
    private SimpleMatrix forwardKinematics;
    private SimpleMatrix inverseKinematics;

    private double m_extensionInitLength;
    private double m_topPivotLength;
    public PinkArmKinematics(double extensionInitLength, double topPivotLength) {
        m_extensionInitLength = extensionInitLength;
        m_topPivotLength = topPivotLength;

        double[][] forwardKinematicInternals = {{1.0, topPivotLength / 2.0}};

        forwardKinematics = new SimpleMatrix(forwardKinematicInternals);

        inverseKinematics = forwardKinematics.pseudoInverse();
        
    }

    public PinkArmState toPinkArmState(Rotation2d bottomAngle, double extension, Rotation2d topAngle) {
        double[][] stateMatrixInternals = {{bottomAngle.getRadians() * extension}, {}};
        SimpleMatrix stateMatrix = new SimpleMatrix();
    }


}
