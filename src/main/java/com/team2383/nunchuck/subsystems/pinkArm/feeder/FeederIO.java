package com.team2383.nunchuck.subsystems.pinkArm.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double current = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {
  }

  /** Run closed loop at the specified duty cycle. */
  public default void setPower(double dutyCycle) {
  }

  /** Stop in open loop. */
  public default void stop() {
  }
}
