// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


@Logged
public class Tourelle extends SubsystemBase {

  //moteur 
  private SparkFlex moteurTourelle = new SparkFlex(71, MotorType.kBrushless); 
  private SparkFlexConfig configTourelle = new SparkFlexConfig(); 
  private double conversionTourelle = 20.0 / 200.0 * 360.0; //moteur gear 20 dents tourelle 200 dents, 360 degrés

  //PID 
  private ProfiledPIDController pidTourelle = new ProfiledPIDController(0, 0, 0, //Valeurs à déterminer
    new TrapezoidProfile.Constraints(0, 0)); 


  public Tourelle() {
    //Moteur + config 
    configTourelle.inverted(false); 
    configTourelle.idleMode(IdleMode.kBrake); 
    configTourelle.encoder.positionConversionFactor(conversionTourelle);
    configTourelle.encoder.velocityConversionFactor(conversionTourelle / 60.0);
    configTourelle.softLimit.forwardSoftLimit(90.0).reverseSoftLimit(-90.0); //Faudrait peut être call la fonction pour l'enable? 
    moteurTourelle.configure(configTourelle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    //PID 
    pidTourelle.setTolerance(0); //À déterminer

  }


  @Override
  public void periodic() {

  }

  public void setVoltage(double voltage){
    moteurTourelle.setVoltage(voltage);
  }

  private void setVoltageHoraire(){
    setVoltage(-2);
  }
  private void setVoltageAntiHoraire(){
    setVoltage(2);
  }

  public void stop(){
    setVoltage(0);
  }

  public double getAngleReel(){
    return moteurTourelle.getEncoder().getPosition(); 
  } 

  public double getAngleAbsolu(){
    if (Math.abs(getAngleReel())< 180.0) {
      return getAngleReel();
    }
    else {
      return (Math.abs(getAngleReel())-360.0) * Math.signum(getAngleReel());
    }
  }


  public double getVitesse(){
    return moteurTourelle.getEncoder().getVelocity(); 
  } 

  public void resetEncoders(){
    moteurTourelle.getEncoder().setPosition(0); 
  }

  //PID 
  public void setPID(double cible){
    double voltagePID = pidTourelle.calculate(getAngleReel(),cible); 
    setVoltage(voltagePID);


  }

  public void resetPID(){
    pidTourelle.reset(getAngleReel());
  }

  public boolean atCible(){
    return pidTourelle.atGoal(); 
  } 


  //Commandes temporaires 
  public Command tournerHoraire(){
    return Commands.runEnd(this::setVoltageHoraire, this::stop, this); 
  }

  public Command tournerAntiHoraire(){
    return Commands.runEnd(this::setVoltageAntiHoraire,this::stop,this); 
  }


}
