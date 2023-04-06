package com.team2383.diffy.subsystems.pinkArm.feeder;

import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;

public final class FeederConstants {
    public static final double kV = 0.01;
    public static final double kA = 0.001;

    public static final int kTopMotorID = 6;
    public static final int kBottomMotorID = 7;

    public static final int MAX_CURRENT = 20;

    public static final class MotorConfigs {
        public TalonFXConfiguration kDriveMotorConfigs;

        public MotorConfigs(double kP, double kI, double kD, double kS, double kV) {
            kDriveMotorConfigs = new TalonFXConfiguration();

            kDriveMotorConfigs.CurrentLimits = new CurrentLimitsConfigs();
            kDriveMotorConfigs.CurrentLimits.SupplyCurrentLimit = 65;
            kDriveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

            kDriveMotorConfigs.Slot0 = new Slot0Configs();
            kDriveMotorConfigs.Slot0.kP = kP;
            kDriveMotorConfigs.Slot0.kI = kI;
            kDriveMotorConfigs.Slot0.kD = kD;

            kDriveMotorConfigs.Slot0.kS = kS;
            kDriveMotorConfigs.Slot0.kV = kV;
            kDriveMotorConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
        }

        public final TalonFXConfiguration feederConfig() {
            return kDriveMotorConfigs;
        }
    }

    // TODO: ARBITRARY VALUES, MUST TUNE WITH SYSID LATER
    public static final MotorConfigs kMotorConfigs = new MotorConfigs(0.1, 0, 0, 0, 0);
}
