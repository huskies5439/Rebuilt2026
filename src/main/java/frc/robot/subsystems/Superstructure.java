// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.Vector;

import com.ctre.phoenix6.sim.ChassisReference;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Cible;
@Logged
public class Superstructure extends SubsystemBase {

 Transform2d deplacementTourelle = new Transform2d(-0.153, -0.153, Rotation2d.kZero);

  private Translation3d cible = new Translation3d();
  private Pose2d poseRobot = new Pose2d(); 
  private ChassisSpeeds chassisSpeedsRobot = new ChassisSpeeds();

  public Superstructure() {
 
  }

  @Override
  public void periodic() {
    
  }

//Mettre en commande par défaut
  public void setCible(boolean isHub){
    if(Constants.isRedAlliance()){
      if(isHub){
        cible = Constants.Cible.hubRouge;
      }else if(poseRobot.getY() > 4 ){
        cible = Constants.Cible.souffleuseOutPostRouge;
      }else{
        cible = Constants.Cible.souffleuseDepotRouge;
      }
    }else{
      if(isHub){
        cible = Constants.Cible.hubBleu;
      }else if(poseRobot.getY() > 4){
        cible = Constants.Cible.souffleuseDepotBleu;
      }else{
        cible = Constants.Cible.souffleuseOutPostBleu;
      }
    }
  }

  public void setPoseRobot(Pose2d poseRobot) {
      this.poseRobot = poseRobot;
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeeds){
    this.chassisSpeedsRobot = chassisSpeeds;
  }

  public Translation2d getVecteurCibleTourelle() {
    Translation2d poseTourelle = poseRobot.plus(deplacementTourelle).getTranslation(); 
    return cible.toTranslation2d().minus(poseTourelle);
  }

  public double getDistanceCibleTourelle(){
    return getVecteurCibleTourelle().getNorm(); 
  }

  
  public Rotation2d getAngleCible() {
    Rotation2d angleVecteurCible = getVecteurCibleTourelle().getAngle();
    Rotation2d angleTourelle = poseRobot.getRotation().rotateBy(Rotation2d.k180deg); 
    return angleVecteurCible.minus(angleTourelle);  
  }

  public double normeVecteurLancer() { 
    double vitesseBallon;
    if (getDistanceCibleTourelle() < 1) {
      vitesseBallon = Constants.RegimeLanceur.vitesseProche;
    } else {
      vitesseBallon = Constants.RegimeLanceur.vitesseLoin; 
    }
    //Ajouter la correction de la vitesse ici sur la vitesse Ballonn

    return vitesseBallon;
  }



  //pitch 
  public double pitchVecteurLancer(Pose2d poseRobot){ //ajouter un clamp pour éviter de briser le hood 
    double vitesseBallon = normeVecteurLancer(); 
    double distance = getDistanceCibleTourelle(); 
    double hauteur = cible.getZ(); 
    return Math.toDegrees(
      Math.atan(
        (Math.pow(vitesseBallon, 2) 
        + Math.sqrt(
            Math.pow(vitesseBallon, 4)
            -Math.pow(Constants.g,2) * Math.pow(distance,2) 
            - 2 * Constants.g * Math.pow(vitesseBallon, 2) * hauteur)) 
        / Constants.g * distance
        )); 
  } 

  //Yaw 
  public double yawVecteurLancer(Pose2d poseRobot){
    return getAngleCible().getDegrees(); //ajouter la correction pour la vitesse ici 
  }

  

}
