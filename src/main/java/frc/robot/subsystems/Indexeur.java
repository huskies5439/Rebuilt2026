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

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexeur extends SubsystemBase {
  /** Creates a new Indexeur. */
  private SparkFlex moteurCarroussel = new SparkFlex(31,MotorType.kBrushless);
  private SparkFlex moteurAccelerateur = new SparkFlex(32,MotorType.kBrushless);
  
  private SparkFlexConfig moteurCarrousselConfig = new SparkFlexConfig();
  private SparkFlexConfig moteurAccelerateurConfig = new SparkFlexConfig();

  public Indexeur() {
    
    moteurCarrousselConfig.inverted(false);
    moteurCarrousselConfig.idleMode(IdleMode.kBrake);
    moteurCarroussel.configure(moteurCarrousselConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    moteurAccelerateurConfig.inverted(false);
    moteurAccelerateurConfig.idleMode(IdleMode.kCoast);
    moteurAccelerateur.configure(moteurCarrousselConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVoltageCaroussel(double voltage) {
    moteurCarroussel.setVoltage(voltage);
  }

  public void stopCaroussel() {
    setVoltageCaroussel(0);
  }

  public void setVoltage(double voltage) {
    moteurAccelerateur.setVoltage(voltage);
  }

  public void stop() {
    setVoltage(0);
  }
}
