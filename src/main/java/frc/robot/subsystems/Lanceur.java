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

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lanceur extends SubsystemBase {
  /** Creates a new Lanceur. */

  private SparkFlex lanceur1 = new SparkFlex(0, MotorType.kBrushless);
  private SparkFlex lanceur2 = new SparkFlex(0, MotorType.kBrushless);
  private SparkFlex lanceurHood = new SparkFlex(0, MotorType.kBrushless);

  private SparkFlexConfig lanceurConfig1 = new SparkFlexConfig();
  private SparkFlexConfig lanceurConfig2 = new SparkFlexConfig();
  private SparkFlexConfig lanceurConfigHood = new SparkFlexConfig();

  private double conversion = 1.0;
  private double conversionHood = 1.0;

  public Lanceur() {
    lanceurConfig1.inverted(false);
    lanceurConfig1.idleMode(IdleMode.kCoast);
    lanceurConfig1.encoder.positionConversionFactor(conversion);
    lanceurConfig1.encoder.velocityConversionFactor(conversion / 60.0);
    lanceur1.configure(lanceurConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    lanceurConfig2.inverted(false);
    lanceurConfig2.idleMode(IdleMode.kCoast);
    lanceurConfig2.encoder.positionConversionFactor(conversion);
    lanceurConfig2.encoder.velocityConversionFactor(conversion / 60.0);
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

  
}
