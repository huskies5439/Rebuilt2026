// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
private SparkMax moteur = new SparkMax(41, MotorType.kBrushless);
  private SparkMaxConfig config = new SparkMaxConfig();
  private double conversion = 1.0;

  public Hood() {

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

  public void stop() {
    moteur.setVoltage(0);
  }

   public void sortir() {
    setVoltage(2);
  }

  public void rentrer() {
    setVoltage(-2);
  }

  public double getPosition() {
    return moteur.getEncoder().getPosition();
  }

  public Command rentrerCommand() {
    return Commands.runEnd(this::rentrer, this::stop, this);
  }

  public Command sortirCommand() {
    return Commands.runEnd(this::sortir, this::stop, this);
  }
}
