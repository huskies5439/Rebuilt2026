// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

 public static final double maxVitesseLineaire = 3.75;// Vitesse linéaire max du chassis 

  public static final double maxVitesseRotation = Math.PI * 1.4; // radians per second

  // public static final double maxVitesseModule = 4.46;// Vitesse maximale d'un module en m/s, pas nécessaire si setpoint generator

  // Chassis configuration
  public static final double kTrackWidth = Units.inchesToMeters(25.5);

  // Distance between centers of right and left wheels on robot
  public static final double kWheelBase = Units.inchesToMeters(25.5);

  // Distance between front and back wheels on robot
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  public static final class PositionHub {
    public Translation2d hubBleu = new Translation2d(4.625,4.025); 
    public Translation2d hubRouge = new Translation2d(11.9,4.025); 
  }

  public static final class PositionYTrench {
    public double trenchBas = 0.65; 
    public double trenchHaut = 7.4; 
  }
    
}
