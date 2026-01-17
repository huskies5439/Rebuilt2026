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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gobeur extends SubsystemBase {
  /** Creates a new Gobbeur. */

  private SparkFlex moteurRouleau = new SparkFlex(41, MotorType.kBrushless);
  private SparkFlex moteurCoude = new SparkFlex(42, MotorType.kBrushless);
  // changer noms pour MoteurQuiDessend et MoteurCouroie

  private SparkFlexConfig moteurRouleauConfig = new SparkFlexConfig();
  private SparkFlexConfig moteurCoudeConfig = new SparkFlexConfig();

  private DigitalInput limitSwitch = new DigitalInput(0);

  private double conversionCoude = 1.0;

  /// PID et feedForward

  private ProfiledPIDController pidCoude = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(1, 1));

  private ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);// valeur à déterminer

  public Gobeur() {
    moteurRouleauConfig.inverted(false);
    moteurRouleauConfig.idleMode(IdleMode.kCoast);
    moteurRouleau.configure(moteurRouleauConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    moteurCoudeConfig.inverted(false);
    moteurCoudeConfig.idleMode(IdleMode.kBrake);
    moteurCoudeConfig.encoder.positionConversionFactor(conversionCoude);
    moteurCoudeConfig.encoder.velocityConversionFactor(conversionCoude / 60.0);

    moteurCoude.configure(moteurCoudeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    resetEncodeurStartUp();

    pidCoude.setTolerance(2);

  }

  @Override
  public void periodic() {

    // SmartDashboard
    SmartDashboard.putNumber("Angle Coude", getAngleCoude());
    // SmartDashboard.putNumber("Vitesse Poignet", getVitesse());
    // SmartDashboard.putNumber("Cible Poignet : ", getCibleManetteOperateur());
    // SmartDashboard.putBoolean("Capteur Poignet", isLimitSwitch());
    // SmartDashboard.putBoolean("Pgn. PID AT CIBLE", atCible());
    SmartDashboard.putBoolean("AtCible Poignet", atCible());

    if (isLimitSwitch()) {

      resetEncodeurLimitSwitch();
    }
  }

  ////////// Rouleau
  public void setVoltageRouleau(double voltage) {
    moteurRouleau.setVoltage(voltage);
  }

  public void stopRouleau() {
    setVoltageRouleau(0);
  }

  public void avaler() {
    setVoltageRouleau(1);
  }

  public void recracher() {
    setVoltageRouleau(-1);
  }

  ////////// Coude

  /// moteur

  public void setVoltageCoude(double voltage) {
    moteurCoude.setVoltage(voltage);
  }

  public void monterCoude() {
    setVoltageCoude(1);
  }

  public void descendreCoude() {
    setVoltageCoude(-1);
  }

  public void stopCoude() {
    setVoltageCoude(0);
  }

  public void holdCoude() {
    setVoltageCoude(feedforward.calculate(Math.toRadians(getAngleCoude()), 0));
  }

  /// Encodeur

  public double getAngleCoude() {
    return moteurCoude.getEncoder().getPosition();
  }

  public double getVitesseCoude() {
    return moteurCoude.getEncoder().getVelocity();
  }

  public void resetEncodeurLimitSwitch() {// Quand on clique la limit switch
    moteurCoude.getEncoder().setPosition(0); // À determiner
  }

  public void resetEncodeurStartUp() {
    moteurCoude.getEncoder().setPosition(0);// à déterminer
  }

  /// PID + feedForward
  public void setPID(double cible) {
    double voltagePIDCoude = pidCoude.calculate(getAngleCoude(), cible);

    double voltageFFCoude = feedforward.calculate(
        Math.toRadians(getAngleCoude()),
        pidCoude.getSetpoint().velocity);
    setVoltageCoude(voltagePIDCoude + voltageFFCoude);

  }

  public void resetPID() {
    pidCoude.reset(getAngleCoude());
  }

  public boolean atCible() {
    return pidCoude.atGoal();
  }

  /// limit switch

  public boolean isLimitSwitch() {
    return !limitSwitch.get();
  }

  /// Commandes

  public Command avalerCommand() {

    return Commands.runEnd(this::avaler, this::stopRouleau, this);
  }

  public Command recracherCommand() {
    return Commands.runEnd(this::recracher, this::stopRouleau, this);
  }

  public Command monterCoudeCommand() {
    return Commands.runEnd(this::monterCoude, this::stopCoude, this);
  }

  public Command descendreCoudeCommand() {
    return Commands.runEnd(this::descendreCoude, this::stopCoude, this);
  }
}
