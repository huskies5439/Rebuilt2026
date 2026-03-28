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
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


@Logged
public class Tourelle extends SubsystemBase {

  //moteur 
  private SparkFlex moteurTourelle = new SparkFlex(52, MotorType.kBrushless); //Max CAN ID = 62
  private SparkFlexConfig configTourelle = new SparkFlexConfig(); 
  private double conversionTourelle = 20.0 / 200.0 * 360.0; //moteur gear 20 dents tourelle 200 dents, 360 degrés

  //PID 
  private double vraieCible = 0.0; 
  private ProfiledPIDController pidTourelle = new ProfiledPIDController(0.05, 0, 0.001, //Valeurs à déterminer
    new TrapezoidProfile.Constraints(720, 1440)); 


  public Tourelle() {
    //Moteur + config 
    configTourelle.inverted(true); 
    configTourelle.idleMode(IdleMode.kBrake); 
    configTourelle.encoder.positionConversionFactor(conversionTourelle);
    configTourelle.encoder.velocityConversionFactor(conversionTourelle / 60.0);
    configTourelle.softLimit.forwardSoftLimit(200.0).reverseSoftLimit(-200.0); //Faudrait peut être call la fonction pour l'enable? 
    configTourelle.softLimit.forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    moteurTourelle.configure(configTourelle, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 

    //PID 
    resetEncoder();
  }


  @Override
  public void periodic() {

  }

  public void setVoltage(double voltage){
    moteurTourelle.setVoltage(voltage);
  }

  private void setVoltageHoraire(){
    setVoltage(-1);
  }
  private void setVoltageAntiHoraire(){
    setVoltage(1);
  }

  public void stop(){
    setVoltage(0);
    resetPID();
  }

  public double getAngleReel(){
    return moteurTourelle.getEncoder().getPosition(); 
  } 

  public double getAngleAbsolu(){
    return MathUtil.inputModulus(getAngleReel(),-180.0,180.0);
  }


  @Logged
  public double getVitesse(){
    return moteurTourelle.getEncoder().getVelocity(); 
  } 

  public void resetEncoder(){
    moteurTourelle.getEncoder().setPosition(0); 
  }

  //Cibles 
  public void setCible(double cible){
    this.vraieCible = cible;
  }

  public double getCible(){
    return vraieCible; 
  }

  //PID 
  public void setPID(double cible){
     setCible(cible);
    double voltagePID = pidTourelle.calculate(getAngleReel(),cible); 
    setVoltage(voltagePID);
  }

  public void resetPID(){
    setCible(getAngleReel());
    pidTourelle.reset(getAngleReel());
  }

  @NotLogged
  public boolean atCible(double toleranceTourelle){
    return Math.abs(getAngleReel() - getCible()) <= toleranceTourelle;  
  } 


  //Commandes temporaires 
  public Command tournerHoraire(){
    return Commands.runEnd(this::setVoltageHoraire, this::stop, this); 
  }

  public Command tournerAntiHoraire(){
    return Commands.runEnd(this::setVoltageAntiHoraire,this::stop,this); 
  }
  public Command PIDCommand(){
    return Commands.runEnd(()->this.setPID(180),this::stop,this);
  }

}
