// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



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

  public static final class Cible {
    public static double hauteurHub = 1.8288 - 0.42995; //Hauteur du hub moins la hauteur de la tourelle relative au sol (mètres)
    public static double hauteurSouffleuse = 0.0;
    public static Translation3d hubBleu = new Translation3d(4.625,4.025,hauteurHub); 
    public static Translation3d hubRouge = new Translation3d(11.9,4.025,hauteurHub);
    public static Translation3d souffleuseDepotBleu = new Translation3d(3.0,6.0,hauteurSouffleuse);
    public static Translation3d souffleuseOutPostBleu = new Translation3d(3.0,2.0,hauteurSouffleuse);
    public static Translation3d souffleuseOutPostRouge = new Translation3d(13.0,6.0,hauteurSouffleuse);
    public static Translation3d souffleuseDepotRouge = new Translation3d(13.0,2.0,hauteurSouffleuse);
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
    public static final double kDLineaire = 1.0;
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

  /////// ALLIANCE
  public static boolean isRedAlliance() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      return ally.get() == Alliance.Red;

    } else {
      return false;
    }
  }

  public static double angleHoodLimitSwitch = 85.0; //à déterminer 

  public static double coefficientFrictionBallon = 1.0;//à déterminer

    
}
