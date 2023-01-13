package com.team2383.diffy.subsystems;
// import com.team2383.diffy.Constants.TelescopeConstants;
// import com.team2383.diffy.helpers.DoubleEncoder;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.revrobotics.CANSparkMax;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.Nat;
// import edu.wpi.first.math.VecBuilder;
// import edu.wpi.first.math.controller.LinearQuadraticRegulator;
// import edu.wpi.first.math.estimator.KalmanFilter;
// import edu.wpi.first.math.numbers.*;
// import edu.wpi.first.math.system.LinearSystem;
// import edu.wpi.first.math.system.LinearSystemLoop;

public class DoublePinkArmSubsystem {

    // private final CANSparkMax pivotMotor;

    // private final CANSparkMax telescopeMotorLeft;
    // private final CANSparkMax telescopeMotorRight;

    // private final TalonFX bottomMotorLeft;
    // private final TalonFX bottomMotorRight;

    // private final DoubleEncoder telescopeEncoderLeft;
    // private final DoubleEncoder telescopeEncoderRight;

    // private final DoubleEncoder bottomAngleEncoder;
    // private final DoubleEncoder topAngleEncoder;

    // private final LinearSystemLoop<N8, N5, N8> m_systemLoop;

    // public DoublePinkArmSubsystem() {
    //     pivotMotor = new CANSparkMax(TelescopeConstants.kTopPivotMotorID, MotorType.kBrushless);

    //     telescopeMotorLeft = new CANSparkMax(TelescopeConstants.kLeftExtensionID, MotorType.kBrushless);
    //     telescopeMotorRight = new CANSparkMax(TelescopeConstants.kRightExtensionID, MotorType.kBrushless);

    //     bottomMotorLeft = new TalonFX(0);
    //     bottomMotorRight = new TalonFX(0);

    //     telescopeMotorLeft.setInverted(false);

    //     telescopeMotorLeft.setIdleMode(IdleMode.kBrake);

    //     telescopeEncoderLeft = new DoubleEncoder(TelescopeConstants.kEncoderPortA, TelescopeConstants.kEncoderPortB, TelescopeConstants.kEncoderPortAbs);
    //     telescopeEncoderRight = new DoubleEncoder(TelescopeConstants.kEncoderPortA, TelescopeConstants.kEncoderPortB, TelescopeConstants.kEncoderPortAbs);

    //     bottomAngleEncoder = new DoubleEncoder(TelescopeConstants.kEncoderPortA, TelescopeConstants.kEncoderPortB, TelescopeConstants.kEncoderPortAbs);
    //     topAngleEncoder = new DoubleEncoder(TelescopeConstants.kEncoderPortA, TelescopeConstants.kEncoderPortB, TelescopeConstants.kEncoderPortAbs);

    //     LinearSystem<N8, N5, N8> armPlant = new LinearSystem<N8, N5, N8>(
    //         Matrix.mat(Nat.N8(), Nat.N8()).fill(
    //             -TelescopeConstants.kV/TelescopeConstants.kA, 0, 0, 0, 0, 0, 0, 0,
    //             0, -TelescopeConstants.kV/TelescopeConstants.kA, 0, 0, 0, 0, 0, 0, 
    //             0, 0, -TelescopeConstants.kV/TelescopeConstants.kA, 0, 0, 0, 0, 0,
    //             0, 0, 0, -TelescopeConstants.kV/TelescopeConstants.kA, 0, 0, 0, 0,
    //             0, 0, 0, 0, -TelescopeConstants.kV/TelescopeConstants.kA, 0, 0, 0,
    //             TelescopeConstants.kgt, 0, 0, 0, 0, 0, 0, 0,
    //             0, TelescopeConstants.kge/2, TelescopeConstants.kge/2, 0, 0, 0, 0, 0,
    //             0, 0, 0, TelescopeConstants.kgb/2, TelescopeConstants.kgb/2, 0, 0, 0),
    //         Matrix.mat(Nat.N8(), Nat.N5()).fill(
    //             1/TelescopeConstants.kA, 0, 0, 0, 0,
    //             0, 1/TelescopeConstants.kA, 0, 0, 0,
    //             0, 0, 1/TelescopeConstants.kA, 0, 0,
    //             0, 0, 0, 1/TelescopeConstants.kA, 0,
    //             0, 0, 0, 0, 1/TelescopeConstants.kA,
    //             0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0),
    //         Matrix.mat(Nat.N8(), Nat.N8()).fill(
    //             1, 0, 0, 0, 0, 0, 0, 0,
    //             0, 1, 0, 0, 0, 0, 0, 0,
    //             0, 0, 1, 0, 0, 0, 0, 0,
    //             0, 0, 0, 1, 0, 0, 0, 0,
    //             0, 0, 0, 0, 1, 0, 0, 0,
    //             0, 0, 0, 0, 0, 1, 0, 0,
    //             0, 0, 0, 0, 0, 0, 1, 0,
    //             0, 0, 0, 0, 0, 0, 0, 1),
    //         Matrix.mat(Nat.N8(), Nat.N5()).fill(
    //             0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0,
    //             0, 0, 0, 0, 0));

    //     LinearQuadraticRegulator<N8, N5, N8> controller = new LinearQuadraticRegulator<>(armPlant,
    //         VecBuilder.fill(1, 1, 1, 1, 1, 0.001, 1, 0.001), VecBuilder.fill(12, 12, 12, 12, 12), 0.02);

    //     KalmanFilter<N8, N5, N8> observer = new KalmanFilter<>(Nat.N8(), Nat.N8(), armPlant,
    //         VecBuilder.fill(0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1), VecBuilder.fill(0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01), 0.02);

    //     m_systemLoop = new LinearSystemLoop<>(armPlant, controller,
    //         observer, 12.0, 0.02);
    // }

    // public void periodic() {
    // }

    // public void setPower(double power) {
    // }
    
    // public double getEncoderPos() {
    //     return 0.0;
    // }
}
