package com.team2383.diffy.helpers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class SparkMaxSimCollection {
    private final SimDevice m_device;
    private SimDouble m_velocity;
    private SimDouble m_position;
    private SimDouble m_voltage;

    public SparkMaxSimCollection(CANSparkMax motor) {
        String deviceKey = "SPARK MAX [" + motor.getDeviceId() + "]";

        m_device = SimDevice.create(deviceKey);

        m_velocity = m_device.createDouble("Velocity", Direction.kOutput, 0.0);

        m_position = m_device.createDouble("Position", Direction.kOutput, 0.0);

        m_voltage = m_device.createDouble("Voltage", Direction.kInput, 0.0);

    }

    /**
     * Sets the simulated voltage output of the motor controller in sensor units.
     * @param velocity
     */
    public void setVelocity(double velocity) {
        m_velocity.set(velocity);
    }

    /**
     * Sets the simulated position of the motor controller in encoder ticks.
     * @param position
     */
    public void setPosition(double position) {
        m_position.set(position);
    }

    public double getSensorVelocity() {
        return m_voltage.get();
    }
    
}
