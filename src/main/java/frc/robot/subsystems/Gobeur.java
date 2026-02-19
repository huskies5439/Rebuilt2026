// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Gobeur extends SubsystemBase {


  private SparkFlex moteur = new SparkFlex(31, MotorType.kBrushless);
 

  private SparkFlexConfig moteurConfig = new SparkFlexConfig();
  

  public Gobeur() {
    moteurConfig.inverted(false);
    moteurConfig.idleMode(IdleMode.kCoast);
    moteur.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
  }

  @Override
  public void periodic() {

  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public void stop() {
    setVoltage(0);
  }

  public void gober() {
    setVoltage(4);
  }

  public void recracher() {
    setVoltage(-1);
  }


  /// Commandes simples

  public Command goberCommand() {

    return Commands.runEnd(this::gober, this::stop, this);
  }

  public Command recracherCommand() {//Pertinent ?????
    return Commands.runEnd(this::recracher, this::stop, this);
  }


}
