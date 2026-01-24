// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lanceur extends SubsystemBase {
  /** Creates a new Lanceur. */

  private SparkFlex moteurGauche = new SparkFlex(61, MotorType.kBrushless);
  private SparkFlex moteurDroit = new SparkFlex(62, MotorType.kBrushless);

  private SparkFlexConfig config = new SparkFlexConfig();

  private double conversionLanceur = 1.0; 

  private PIDController pid = new PIDController(0, 0, 0); //les valeurs sont permanentes 

  public Lanceur() {
    boolean inverted = false;
    config.inverted(inverted);
    config.idleMode(IdleMode.kCoast);
    config.encoder.positionConversionFactor(conversionLanceur);
    config.encoder.velocityConversionFactor(conversionLanceur / 60.0);
    moteurGauche.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(!inverted);
    moteurDroit.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltage(double voltage) {
    moteurGauche.setVoltage(voltage);
    moteurDroit.setVoltage(voltage);
  }

  public void lancer() {
    setVoltage(1);
  }

  public void stop() {
    setVoltage(0);
  }

  //PID 
  public void setPID(double cible){
    double voltage = pid.calculate(cible);
    setVoltage(voltage);
  }

  public boolean atCible(){
    return pid.atSetpoint(); 
  }

  public void resetPID(){
    pid.reset();
  }

  public double getPosition() {
    return (moteurGauche.getEncoder().getPosition() + moteurDroit.getEncoder().getPosition()) / 2.0;
  }

  public double getVitesse() {
    return (moteurGauche.getEncoder().getVelocity() + moteurDroit.getEncoder().getVelocity()) / 2.0;
  }

  public Command lancerSimpleCommand() {
    return Commands.runEnd(this::lancer, this::stop, this);
  }

  public Command lancerPIDCommand(double cible) {
    return Commands.runEnd(()->setPID(cible), this::stop, this);
  }

}
