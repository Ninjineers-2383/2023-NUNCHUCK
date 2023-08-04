package com.team2383.nunchuck.helpers;

import edu.wpi.first.wpilibj.Timer;

/**
 * A class to help with velocity calculations
 */
public class LinearVelocityWrapper {
    private double lastDisplacement;
    private double lastTime;
    private double velocity = 0;

    public LinearVelocityWrapper() {
        this(0);
    }

    public LinearVelocityWrapper(double initialPosition) {
        lastDisplacement = initialPosition;
        lastTime = Timer.getFPGATimestamp();
    }

    /**
     * Calculates the velocity of a system given a displacement
     * Need to run this every loop to be accurate
     * 
     * @param displacement
     * @return discrete-time-derivitive velocity
     */
    public double calculate(double displacement) {
        double deltaTime = Timer.getFPGATimestamp() - lastTime;
        velocity = (displacement - lastDisplacement) / (deltaTime - lastTime);
        lastDisplacement = displacement;
        lastTime = deltaTime;
        return velocity;
    }

    /** Returns velocity value without recalculating */
    public double get() {
        return velocity;
    }
}