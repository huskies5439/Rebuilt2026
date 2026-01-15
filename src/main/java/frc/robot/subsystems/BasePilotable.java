// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BasePilotable extends SubsystemBase {
  // Créer les moteurs swerves
  private MAXSwerveModule avantGauche = new MAXSwerveModule(1, 2, -90);

  private MAXSwerveModule avantDroite = new MAXSwerveModule(3, 4, 0);

  private MAXSwerveModule arriereGauche = new MAXSwerveModule(5, 6, 180);

  private MAXSwerveModule arriereDroite = new MAXSwerveModule(7, 8, 90);

  // Le gyroscope
  private Pigeon2 gyro = new Pigeon2(0);

  private PPHolonomicDriveController ppHolonomicDriveController = new PPHolonomicDriveController(
      new PIDConstants(2, 0, 0), // valeur stupide de 12 a Montréal ; ne plus faire l'Erreur S.V.P
      new PIDConstants(10, 0, 1));

  /*
   * Setpoint genetator est une fonction de PathPlanner qui permet de valider
   * si les vitesses demandées en téléop respectent les contraintes mécaniques
   * du robot telles que définies dans les App Settings de PathPlanner
   * C'est expérimental en 2025, donc valider les changements pour l'an prochain
   */

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  // Initialisation PoseEstimator
  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      Constants.kDriveKinematics,
      Rotation2d.fromDegrees(getAngle()),
      new SwerveModulePosition[] {
          avantGauche.getPosition(),
          avantDroite.getPosition(),
          arriereGauche.getPosition(),
          arriereDroite.getPosition()
      },
      Pose2d.kZero);

  Field2d field2d = new Field2d();

  public BasePilotable() {
    // Reset initial
    resetGyro();
    resetEncoders();
    resetOdometry(new Pose2d());

    // aller chercher la configuration du robot dans Pathplanner
    RobotConfig robotConfig = null;
    try {

      robotConfig = RobotConfig.fromGUISettings();

    } catch (Exception e) {
      e.printStackTrace();
    }
    // configuration Pathplanner
    // Voir la documentation, ça va peut-être changer en 2026

    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, feedforward) -> conduireChassis(speeds),
        ppHolonomicDriveController,
        robotConfig,
        this::isRedAlliance,
        this);

    // Configuration Setpoint Generator
    setpointGenerator = new SwerveSetpointGenerator(
        robotConfig, Units.rotationsToRadians(3.94) // Valeur selon la freespeed du neo 550
    );

    resetSetpoint();
  }

  @Override
  public void periodic() {

  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // La librairie de REV utilise la fonction .desaturate ici.
    // Attention, ils utilisent le maxChassisSpeed au lieu du maxVitesseModule
    // SetPointGenerator ôte la nécessiter de désaturer
    avantGauche.setDesiredState(desiredStates[0]);
    avantDroite.setDesiredState(desiredStates[1]);
    arriereGauche.setDesiredState(desiredStates[2]);
    arriereDroite.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        avantGauche.getState(), avantDroite.getState(), arriereGauche.getState(), arriereDroite.getState()
    };
  }

  ////////////// Encodeurs
  // Pas besoin de méthode pour obtenir la position des encodeurs, tout ça
  // passe directement par la pose2D du robot
  public void resetEncoders() {
    avantGauche.resetEncoders();
    arriereGauche.resetEncoders();
    avantDroite.resetEncoders();
    arriereDroite.resetEncoders();
  }

  /////////////// GYRO
  public double getAngle() {
    return gyro.getYaw().getValueAsDouble();
  }

  public double getRate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }

  public void resetOdometry(Pose2d pose) { // pose est à la pose où reset, c'est typiquement l'origine du terrain
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        new SwerveModulePosition[] {
            avantGauche.getPosition(),
            avantDroite.getPosition(),
            arriereGauche.getPosition(),
            arriereDroite.getPosition()
        },
        pose);
  }

  public void resetSetpoint() {
    // Il faut le caller à chaque fois que l'on retourne à la commande de conduite
    // téléop
    // Sinon les premiers mouvements du robot dépendent de la conduite avant la
    // commande de pathfinding
    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
  }

  /// ///////////// Path Planner
  public ChassisSpeeds getChassisSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(
        avantDroite.getState(), avantGauche.getState(), arriereDroite.getState(), arriereGauche.getState());
  }

   public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

       public void conduireChassis(ChassisSpeeds chassisSpeeds) {
        // Ramene la vitesse en intervale de 20 ms
        ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        SwerveModuleState[] swerveModuleState = Constants.kDriveKinematics.toSwerveModuleStates(targetSpeed);
        setModuleStates(swerveModuleState);
    }

        public boolean isRedAlliance() {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()) {
            return ally.get() == Alliance.Red;

        } else {
            return false;
        }
    }
}
