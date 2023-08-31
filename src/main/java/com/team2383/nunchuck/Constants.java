// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.nunchuck;

/**
 * Note: Translations calculated with the formula graphed here
 * https://www.desmos.com/calculator/jj9i5bdjhe
 * <p>
 * The circle function is used in
 * {@link com.team2383.nunchuck.commands.JoystickDriveCommand#getCenterOfRotation
 * JoystickDriveCommand}
 */

public final class Constants {
    public static final String kRIOBus = "rio";
    public static final String kCANivoreBus = "Drive";

      public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

    public static final class OI {
        // Axis
        public static int DriveX = Robot.isReal() ? 5 : 0;
        public static int DriveY = Robot.isReal() ? 4 : 1;
        public static int DriveOmega = Robot.isReal() ? 0 : 2;
        public static int IntakeIn = Robot.isReal() ? 3 : 3;
        public static int IntakeOut = Robot.isReal() ? 2 : 4;

        // Buttons
        public static int FieldCentric = Robot.isReal() ? 6 : 1;
        public static int ResetHeading = Robot.isReal() ? 8 : 2;
    }
}
