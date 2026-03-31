// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Set;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(strategy = Strategy.OPT_IN)
public class Lanceur extends SubsystemBase {

  private SparkFlex moteurGauche = new SparkFlex(61, MotorType.kBrushless);
  private SparkFlex moteurDroit = new SparkFlex(62, MotorType.kBrushless);

  private SparkClosedLoopController pidFlex = moteurGauche.getClosedLoopController();

  private SparkFlexConfig config = new SparkFlexConfig();

  private double conversionLanceur = 1.0;

  private PIDController pid = new PIDController(0.2, 0, 0.002);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.1, 0.108);

  private SlewRateLimiter limiter = new SlewRateLimiter(200); // Pour limiter l'accélération du lanceur

  private double vraieCible = 0.0;

  private double toleranceLanceur = 3.0;

  public Lanceur() {
    boolean inverted = true;
    config.inverted(inverted);
    config.idleMode(IdleMode.kCoast);

    config.encoder.positionConversionFactor(conversionLanceur);
    config.encoder.velocityConversionFactor(conversionLanceur / 60.0);
    config.encoder.quadratureMeasurementPeriod(10);
    config.encoder.quadratureAverageDepth(2);

    //Les gains PID des Flex sont en duty cycle (entre 0-1), donc il faut diviser par 12...
    config.closedLoop.p(0.2/12.0).i(0).d(0.002/12.0).outputRange(-1, 1);
    //Les gains FF des Flex sont en Volts, donc on est ok avec nos pratiques actuelles
    config.closedLoop.feedForward.kS(0.1).kV(0.108);

    //////ALLER ACTIVER LE PID FLEX DANS SETPID POUR TESTER !!!

    moteurGauche.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(!inverted);
    //////Ajouter le follow pour le pidFlex
    //config.follow(moteurGauche);//On peut aussi mettre un boolean pour que le follow soit automatiquement inversé 
    moteurDroit.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("voltage lanceur", 0);// Initialise input open loop dans le dashboard
    SmartDashboard.putNumber("cible lanceur", 28.5);// Initialise input PID dans le dashboard
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  ///// Moteur
  public void setVoltage(double voltage) {
    moteurGauche.setVoltage(voltage);
    moteurDroit.setVoltage(voltage);
  }

  public void lancer() {
    setVoltage(SmartDashboard.getNumber("voltage lanceur", 0));
  }

  public void stop() {
    setVoltage(0);
    resetPID();
  }

  /////// Encodeur
  public double getPosition() {
    return (moteurGauche.getEncoder().getPosition() + moteurDroit.getEncoder().getPosition()) / 2.0;
  }

  @Logged(name = "Vitesse Lanceur")
  public double getVitesse() {
    return (moteurGauche.getEncoder().getVelocity() + moteurDroit.getEncoder().getVelocity()) / 2.0;
  }

  /////////// PID

  // vraie cible pour déterminer si le lanceur est vraiment à la cible plutot que
  // celle corrigée
  public void setVraieCible(double cible) {
    vraieCible = cible;
  }

  public double getVraieCible() {
    return vraieCible;
  }

  public void setPID(double cible) {
    cible = MathUtil.clamp(cible, 0, 100);
    setVraieCible(cible);
    double cibleCorriger = limiter.calculate(cible);
    setVoltage(
        ff.calculate(cibleCorriger) + pid.calculate(getVitesse(), cibleCorriger));
    //pidFlex.setSetpoint(cibleCorriger, ControlType.kVelocity);
  }

  @Logged(name = "At Cible Lanceur")
  public boolean atCible() {
    return Math.abs(getVitesse() - getVraieCible()) <= toleranceLanceur;
  }

  public void resetPID() {
    setVraieCible(0);
    pid.reset();
    //pidFlex ne semble pas avoir besoin d'un reset
  }

  ///////////////// COMMANDES

  public Command lancerSimpleCommand() {
    return Commands.runEnd(this::lancer, this::stop, this);
  }

  public Command lancerPIDCommand(double cible) {
    return Commands.runOnce(() -> limiter.reset(getVitesse()))
        .andThen(Commands.runEnd(() -> setPID(cible), this::stop, this));
  }

  public Command lancerPIDCommand() {// Version Dashboard
    return Commands.defer(() -> {
      return lancerPIDCommand(SmartDashboard.getNumber("cible lanceur", 0));
    }, Set.of(this));
  }

}
