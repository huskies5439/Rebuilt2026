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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Lanceur extends SubsystemBase {

  private SparkFlex moteurGauche = new SparkFlex(61, MotorType.kBrushless);
  private SparkFlex moteurDroit = new SparkFlex(62, MotorType.kBrushless);

  private SparkFlexConfig config = new SparkFlexConfig();

  private double conversionLanceur = 1.0;

  private PIDController pid = new PIDController(0.1, 0, 0.001);
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.1, 0.108); 
                                                                     
  private SlewRateLimiter limiter = new SlewRateLimiter(200); // Pour limiter l'accélération du lanceur

  private double vraieCible = 0.0;

  public Lanceur() {
    boolean inverted = false;
    config.inverted(inverted);
    config.idleMode(IdleMode.kCoast);
    config.encoder.positionConversionFactor(conversionLanceur);
    config.encoder.velocityConversionFactor(conversionLanceur / 60.0);
    moteurGauche.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.inverted(!inverted);
    moteurDroit.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SmartDashboard.putNumber("voltage lanceur", 0);//Initialise input open loop dans le dashboard
    SmartDashboard.putNumber("cible lanceur", 45);//Initialise input PID dans le dashboard
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /////Moteur
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


  ///////Encodeur
  public double getPosition() {
    return (moteurGauche.getEncoder().getPosition() + moteurDroit.getEncoder().getPosition()) / 2.0;
  }


  @Logged (name = "Vitesse Lanceur")
  public double getVitesse() {
    return (moteurGauche.getEncoder().getVelocity() + moteurDroit.getEncoder().getVelocity()) / 2.0;
  }


  ///////////PID

  // vraie cible pour déterminer si le lanceur est vraiment à la cible plutot que
  // celle corrigée
  public void setVraieCible(double cible) {
    vraieCible = cible;
  }

  public double getVraieCible() {
    return vraieCible;
  }

  public void setPID(double cible) {
    setVraieCible(cible);
    double cibleCorriger = limiter.calculate(cible);
    setVoltage(
        ff.calculate(cibleCorriger) + pid.calculate(getVitesse(), cibleCorriger));
  }

  @Logged (name = "Cible PID Lanceur")
  public boolean atCible() {
    return Math.abs(getVitesse() - getVraieCible()) <= 1; // 1 RPS
  }

  public void resetPID() {
    setVraieCible(0);
    pid.reset();
  }

  
  /////////////////COMMANDES

  public Command lancerSimpleCommand() {
    return Commands.runEnd(this::lancer, this::stop, this);
  }

  public Command lancerPIDCommand(double cible) {
    return Commands.runOnce(() -> limiter.reset(getVitesse()))
        .andThen(Commands.runEnd(() -> setPID(cible), this::stop, this));
  }

  public Command lancerPIDCommand() {//Version Dashboard
      return lancerPIDCommand(SmartDashboard.getNumber("cible lanceur", 0));
  }

}
