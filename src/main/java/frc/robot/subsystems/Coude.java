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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Coude extends SubsystemBase {

  private SparkMax moteurGauche = new SparkMax(21, MotorType.kBrushless);
  private SparkMax moteurDroit = new SparkMax(22, MotorType.kBrushless);

  private SparkMaxConfig moteurConfig = new SparkMaxConfig();

  private DigitalInput limitSwitch = new DigitalInput(0);

  private double conversionCoude = (1 / 5.0) * (1 / 5.0) * (1 / 3.0) * 360.0;

  /// PID et feedForward

  private ProfiledPIDController pidGauche = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(1, 1));

  private ProfiledPIDController pidDroit = new ProfiledPIDController(0, 0, 0,
      new TrapezoidProfile.Constraints(1, 1));

  private ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.0, 0.0);// valeur à déterminer

  /** Creates a new Coude. */
  public Coude() {

    boolean inverted = false; 
    moteurConfig.inverted(inverted);
    moteurConfig.idleMode(IdleMode.kBrake);
    moteurConfig.encoder.positionConversionFactor(conversionCoude);
    moteurConfig.encoder.velocityConversionFactor(conversionCoude / 60.0);

    moteurGauche.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    moteurConfig.inverted(!inverted);

    moteurDroit.configure(moteurConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    resetEncodeurStartUp();

    pidGauche.setTolerance(2);
    pidDroit.setTolerance(2);

  }

  @Override
  public void periodic() {
    // TODO : mettre les capteurs dans le dashboard

    if (isLimitSwitch()) {// TODO :à remplacer par un trigger dans le robotcontainer

      resetEncodeurLimitSwitch();
    }
  }

  public void setVoltage(double voltageGauche, double voltageDroit) {
    moteurGauche.setVoltage(voltageGauche);
    moteurDroit.setVoltage(voltageDroit);
  }

  public void setVoltage(double voltage) {
    setVoltage(voltage, voltage);
  }

  public void monter() {
    setVoltage(1);
  }

  public void descendre() {
    setVoltage(-1);
  }

  public void stop() {
    setVoltage(0);
  }

  public void hold() {
    setVoltage(feedforward.calculate(Math.toRadians(getAngleDroit()), 0));
  }

  /// Encodeur Gauche

  public double getAngleGauche() {
    return moteurGauche.getEncoder().getPosition();
  }

  public double getVitesseGauche() {
    return moteurGauche.getEncoder().getVelocity();
  }

  /// Encodeur Droit

  public double getAngleDroit() {
    return moteurDroit.getEncoder().getPosition();
  }

  public double getVitesseDroit() {
    return moteurDroit.getEncoder().getVelocity();
  }

  /// Encodeurs

  public void resetEncodeurLimitSwitch() {// Quand on clique la limit switch
    moteurDroit.getEncoder().setPosition(0); // À determiner
    moteurGauche.getEncoder().setPosition(0); // À determiner
  }

  public void resetEncodeurStartUp() {
    moteurDroit.getEncoder().setPosition(90);// à déterminer
    moteurGauche.getEncoder().setPosition(90);// à déterminer
  }

  /// PID + feedForward
  public void setPID(double cible) {
    double voltagePIDDroit = pidDroit.calculate(getAngleDroit(), cible);

    double voltageFFDroit = feedforward.calculate(
        Math.toRadians(getAngleDroit()),
        pidDroit.getSetpoint().velocity);

    double voltagePIDGauche = pidGauche.calculate(getAngleGauche(), cible);

    double voltageFFGauche = feedforward.calculate(
        Math.toRadians(getAngleGauche()),
        pidGauche.getSetpoint().velocity);

    setVoltage(voltagePIDGauche + voltageFFGauche, voltagePIDDroit + voltageFFDroit);

  }

  public void resetPID() {
    pidDroit.reset(getAngleDroit());
    pidGauche.reset(getAngleGauche());
  }

  public boolean atCible() {
    return pidDroit.atGoal() && pidGauche.atGoal();
  }

  /// limit switch

  public boolean isLimitSwitch() {
    return !limitSwitch.get();
  }

  public Command monterCommand() {
    return Commands.runEnd(this::monter, this::stop, this);
  }

  public Command descendreCommand() {
    return Commands.runEnd(this::descendre, this::stop, this);
  }

}
