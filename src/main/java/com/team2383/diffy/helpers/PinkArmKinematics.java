package com.team2383.diffy.helpers;

import org.ejml.simple.SimpleMatrix;

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


}
