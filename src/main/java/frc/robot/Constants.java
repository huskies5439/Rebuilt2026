// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;



public final class Constants {

 public static final double maxVitesseLineaire = 3.75;
  public static final double maxVitesseRotation = Math.toRadians(540);

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
    public static Translation2d hubBleu = new Translation2d(4.625,4.025); 
    public static Translation2d hubRouge = new Translation2d(11.9,4.025);
    public static Translation2d souffleuseDepotBleu = new Translation2d(3.0,6.0);
    public static Translation2d souffleuseOutPostBleu = new Translation2d(3.0,2.0);
    public static Translation2d souffleuseOutPostRouge = new Translation2d(13.0,6.0);
    public static Translation2d souffleuseDepotRouge = new Translation2d(13.0,2.0);
  }

  public static final class RegimeLanceur{ //valeurs à déterminer
    public static double distance = 2.0; 
    public static double vitesseProche = 2000; 
    public static double vitesseLoin = 4000; 
    public static double facteurAngleProche = 30.0; 
    public static double facteurAngleLoin = 15.0; 
  }

  public static final class PoseTrench {

    public static final Translation2d trenchBleuDepot = new Translation2d(4.65, 7.4);
    public static final Translation2d trenchBleuOutpost = new Translation2d(4.65, 0.65);
    public static final Translation2d trenchRougeDepot = new Translation2d(11.93, 7.4);
    public static final Translation2d trenchRougeOutpost = new Translation2d(11.93, 0.65);


  }

  public static final double kAngleCoudeDepart = -5; //140.0 ou -8.0 

  public static final double kAngleHoodDepart = 74.0; 

  public static final class PidBasePilotable {
    public static final double kPLineaire = 2.0;
    public static final double kDLineaire = 1.0;
    public static final double kPRot = 10.0;
    public static final double kDRot = 1.0;

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

  public static double coefficientFrictionBallon = 1.0;//à déterminer

    
}
