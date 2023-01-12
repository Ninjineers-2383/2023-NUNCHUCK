package com.team2383.diffy.subsystems;
import com.team2383.diffy.Constants.TelescopeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TelescopeSubsystem {

    private final CANSparkMax telescopeMotor;
    private final Encoder telescopeEncoder;

    public TelescopeSubsystem() {
        telescopeMotor = new CANSparkMax(TelescopeConstants.kMotorID, MotorType.kBrushless);

        telescopeMotor.setInverted(false);

        telescopeMotor.setIdleMode(IdleMode.kBrake);

        telescopeEncoder = new Encoder(TelescopeConstants.kEncoderPortA, TelescopeConstants.kEncoderPortB);

    }

    public void periodic() {
        double telescopeMotorSpeed = telescopeMotor.get(); // Speed from -1.0 to 1.0 
        double telescopeEncoderPos = telescopeEncoder.get(); // Telescoping Encoder pos

        SmartDashboard.putNumber("Telescope Motor Speed: ", telescopeMotorSpeed);
        SmartDashboard.putNumber("Encoder Pos", telescopeEncoderPos);
    }

    public void setPower(double power) {
        telescopeMotor.set(power);
    }
    
    public double getEncoderPos() {
        return telescopeEncoder.get();
    }
}
