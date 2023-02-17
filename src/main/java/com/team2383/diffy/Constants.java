// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team2383.diffy;
/**
 * Note: Translations calculated with the formula graphed here
 * https://www.desmos.com/calculator/jj9i5bdjhe
 * <p>
 * The circle function is used in
 * {@link com.team2383.diffy.commands.JoystickDriveCommand#getCenterOfRotation
 * JoystickDriveCommand}
 */

public final class Constants {
    public static final String kRIOBus = "rio";
    public static final String kCANivoreBus = "Drive";

    public static final class OI {
        public static int DriveX = Robot.isReal() ? 5 : 0;
        public static int DriveY = Robot.isReal() ? 4 : 0;
        public static int DriveOmega = Robot.isReal() ? 0 : 0;
    }
}
