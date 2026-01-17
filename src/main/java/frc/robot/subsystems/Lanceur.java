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

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lanceur extends SubsystemBase {
  /** Creates a new Lanceur. */

  private SparkFlex lanceur1 = new SparkFlex(11, MotorType.kBrushless);
  private SparkFlex lanceur2 = new SparkFlex(12, MotorType.kBrushless);
  private SparkMax lanceurHood = new SparkMax(13, MotorType.kBrushless);

  private SparkFlexConfig lanceurConfig1 = new SparkFlexConfig();
  private SparkFlexConfig lanceurConfig2 = new SparkFlexConfig();
  private SparkMaxConfig lanceurConfigHood = new SparkMaxConfig();

  private double conversionLanceur = 1.0;
  private double conversionHood = 1.0;

  public Lanceur() {
    lanceurConfig1.inverted(false);
    lanceurConfig1.idleMode(IdleMode.kCoast);
    lanceurConfig1.encoder.positionConversionFactor(conversionLanceur);
    lanceurConfig1.encoder.velocityConversionFactor(conversionLanceur / 60.0);
    lanceur1.configure(lanceurConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lanceurConfig2.inverted(false);
    lanceurConfig2.idleMode(IdleMode.kCoast);
    lanceurConfig2.encoder.positionConversionFactor(conversionLanceur);
    lanceurConfig2.encoder.velocityConversionFactor(conversionLanceur / 60.0);
    lanceur2.configure(lanceurConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lanceurConfigHood.inverted(false);
    lanceurConfigHood.idleMode(IdleMode.kCoast);
    lanceurConfigHood.encoder.positionConversionFactor(conversionHood);
    lanceurConfigHood.encoder.velocityConversionFactor(conversionHood / 60.0);
    lanceurHood.configure(lanceurConfigHood, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltageLanceur(double voltage) {
    lanceur1.setVoltage(voltage);
    lanceur2.setVoltage(voltage);
  }

  public void lancer() {
    setVoltageLanceur(1);
  }

  public void stop() {
    setVoltageLanceur(0);
  }

  public void setVoltageHood(double voltage) {
    lanceurHood.setVoltage(voltage);
  }

  public void stopHood() {
    lanceurHood.setVoltage(0);
  }

  public double getPositionLanceur() {
    return (lanceur1.getEncoder().getPosition() + lanceur2.getEncoder().getPosition()) / 2.0;
  }

  public double getPositionHood() {
    return lanceurHood.getEncoder().getPosition();
  }

  public double getVitesseLanceur1() {
    return lanceur1.getEncoder().getVelocity();
  }

  public double getVitesseLanceur2() {
    return lanceur2.getEncoder().getVelocity();
  }

  public Command lancerSimpleCommand() {
    return Commands.runEnd(this::lancer, this::stop, this);
  }
}
