// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kickeur extends SubsystemBase {
  private SparkFlex moteur = new SparkFlex(51, MotorType.kBrushless);
  private SparkFlexConfig config = new SparkFlexConfig();

  public Kickeur() {
    config.inverted(false);
    config.idleMode(IdleMode.kCoast);
    moteur.configure(config, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }
  public void setVoltage(double voltage) {
    moteur.setVoltage(voltage);
  }

  public void tourner() {
    setVoltage(2);
  }

  public void stop() {
    setVoltage(0);
  }

  public Command tournerCommand() {
    return Commands.runEnd(this::tourner, this::stop, this);
  }
}
