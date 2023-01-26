package com.team2383.diffy.helpers;

import com.revrobotics.CANSparkMax;

public class Ninja_CANSparkMax extends CANSparkMax {
    public Ninja_CANSparkMax(int deviceID, MotorType type) {
        super(deviceID, type);
    }
    
    public SparkMaxSimCollection getSimCollection() {
        return new SparkMaxSimCollection(this);
    }
}
