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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Carroussel extends SubsystemBase {

  private SparkFlex moteur = new SparkFlex(11, MotorType.kBrushless);

  private SparkFlexConfig config = new SparkFlexConfig();

  // moteur avec maxPlanetary, poulie 36 vers 72, 360 degrées
  private double maxPlanetary = 9.0;
  private double conversion = (1.0 / maxPlanetary) * (36.0 / 72.0) * 360;

  public Carroussel() {

    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.encoder.positionConversionFactor(conversion);
    config.encoder.velocityConversionFactor(conversion / 60.0);
    moteur.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  
  }

  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public void tournerHoraire() {
    setVoltage(7);
  }

  public void stop() {
    setVoltage(0);
  }

  ////// Commandes

  public Command tournerCommand() {
    return Commands.runEnd(this::tournerHoraire, this::stop, this);
  }


}
