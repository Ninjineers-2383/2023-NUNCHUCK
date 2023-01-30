package com.team2383.diffy.helpers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.SimDevice.Direction;

public class Ninja_CANSparkMax extends CANSparkMax {
    private SimDevice m_device;
    private SimDouble m_velocity;
    private SimDouble m_position;
    private SimDouble m_voltage;

    public Ninja_CANSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);

        m_device = SimDevice.create("Spark Max", this.getDeviceId());

        System.out.println(m_device);

        if (m_device != null) {
            System.out.println("Success");
            m_position = m_device.createDouble("Position", Direction.kOutput, 0.0);

            m_velocity = m_device.createDouble("Velocity", Direction.kOutput, 0.0);

            m_voltage = m_device.createDouble("Voltage", Direction.kInput, 0.0);
            
        }
    }
    
    @Override
    public void set(double speed) {
        if (m_device != null) {
            m_velocity.set(speed);
        } else {
            super.set(speed);
        }
    }

    @Override
    public double get() {
        if (m_device != null) {
            return m_velocity.get();
        } else {
            return super.get();
        }
    }

    @Override
    public void setVoltage(double voltage) {
        if (m_device != null) {
            m_voltage.set(voltage);
        } else {
            super.setVoltage(voltage);
        }
    }
    
    public void setPosition(double pos) {
        if (m_device != null) {
            m_position.set(pos);
        }
    }

    public double getPosition() {
        if (m_device != null) {
            return m_position.get();
        }
        return 0.0;
    }
}
