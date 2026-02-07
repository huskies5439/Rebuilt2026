// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;



public final class Constants {

 public static final double maxVitesseLineaire = 3.75;// Vitesse linéaire max du chassis 

  public static final double maxVitesseRotation = Math.PI * 1.4; // radians per second

  // public static final double maxVitesseModule = 4.46;// Vitesse maximale d'un module en m/s, pas nécessaire si setpoint generator

  // Chassis configuration
  public static final double kTrackWidth = 0.5972 ;

  // Distance between centers of right and left wheels on robot
  public static final double kWheelBase = 0.5972;

  // Distance between front and back wheels on robot
  public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
      new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

  public static final class PositionHub {
    public static Translation2d hubBleu = new Translation2d(4.625,4.025); 
    public static Translation2d hubRouge = new Translation2d(11.9,4.025); 
  }

  public static final class RegimeLanceur{ //valeurs à déterminer
    public static double distance = 2.0; 
    public static double vitesseProche = 2000; 
    public static double vitesseLoin = 4000; 
    public static double facteurAngleProche = 30.0; 
    public static double facteurAngleLoin = 15.0; 
  }

  public static final class PositionYTrench {
    public static final double trenchBas = 0.65; 
    public static final double trenchHaut = 7.4; 
  }

  public static final double kAngleCoudeDepart = 90.0;

  public static final class PidBasePilotable {
    public static final double kPLineaire = 2.0;
    public static final double kPRot = 10.0;
    public static final double kDRot = 1.0;
    //////Si possible, remplacer ces variables par une fonctions qui va chercher les paramètres PP.
    public static final double kMaxVitesseLineaire = 3.0;
    public static final double kMaxAccelLineaire = 1.5;
    public static final double kMaxVitesseRot = Math.toRadians(360);
    public static final double kMaxAccelRot = Math.toRadians(360);
  }

  public static final double g  = 9.81; 

  public static double hauteurHub = 0.42995; // mètres 
    
}
