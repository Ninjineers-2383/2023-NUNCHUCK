package com.team2383.diffy.helpers;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class DoubleEncoder {
    private final Encoder quadEncoder;
    private final EncoderSim quadEncoderSim;
    private final DutyCycleEncoder absEncoder;
    private final DutyCycleEncoderSim absEncoderSim;

    private double m_zeroOffset = 0;

    /**
     * Creates a new DoubleEncoder.s
     * 
     * @param quadChannelA The DIO channel of the quadrature encoder's A channel
     * @param quadChannelB The DIO channel of the quadrature encoder's B channel
     * @param absChannel   The DIO channel of the absolute encoder's PWM channel
     */
    public DoubleEncoder(int quadChannelA, int quadChannelB, int absChannel) {
        quadEncoder = new Encoder(quadChannelA, quadChannelB, true, Encoder.EncodingType.k4X);
        absEncoder = new DutyCycleEncoder(absChannel);

        // Sim encoders to allow simulation of the robot
        quadEncoderSim = new EncoderSim(quadEncoder);
        absEncoderSim = new DutyCycleEncoderSim(absEncoder);

        quadEncoder.setDistancePerPulse(360.0 / 2048.0); // Encoder is mounted on turning shaft so the encoder spins 1:1
                                                         // with the wheel, 360 degrees per 2048 ticks (1 rotation)
    }

    /**
     * Reset the encoder position to zero.
     * Also sets the offset accordingly.
     */
    public void reset() {
        absEncoder.setPositionOffset(0);
        quadEncoder.reset();

        setZeroOffset();
    }

    /**
     * Set the zero offset of the encoder.
     * This is the offset between the absolute and quadrature encoders.
     */
    public void setZeroOffset() {
        double quad = quadEncoder.getDistance();
        double abs = absEncoder.getAbsolutePosition() * 360;
        m_zeroOffset = quad - abs;
        DataLogManager.log(String.format("Set Zero Offset %f\nQuad: %f, Abs: %f", m_zeroOffset, quad, abs));
    }

    /**
     * Simulates the encoder position
     * 
     * @param angle the angle to simulate in degrees
     */
    public void simulate(double angle) {
        absEncoderSim.setDistance(angle);
        quadEncoderSim.setDistance(angle);
    }

    /**
     * Gets the angle of the encoder
     * 
     * @return the angle in degrees
     */
    public double get() {
        return quadEncoder.getDistance() - m_zeroOffset;
    }

    /**
     * Gets the raw value of the quadrature encoder
     * 
     * @return the quadrature ticks
     */
    public int getRawQuad() {
        return quadEncoder.getRaw();
    }

    /**
     * Gets the raw value of the absolute encoder
     * 
     * @return the absolute position
     */
    public double getRawAbs() {
        return absEncoder.getAbsolutePosition();
    }

    /**
     * Gets the zero offset of the encoder
     * 
     * @return the zero offset
     */
    public double getZeroOffset() {
        return m_zeroOffset;
    }
}