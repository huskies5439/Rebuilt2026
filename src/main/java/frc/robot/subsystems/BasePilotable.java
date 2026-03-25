// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PidBasePilotable;
import frc.robot.LimelightHelpers;

@Logged
public class BasePilotable extends SubsystemBase {

  // Créer les moteurs swerves
  private MAXSwerveModule avantGauche = new MAXSwerveModule(1, 2, -90);

  private MAXSwerveModule avantDroite = new MAXSwerveModule(3, 4, 0);

  private MAXSwerveModule arriereGauche = new MAXSwerveModule(5, 6, 180);

  private MAXSwerveModule arriereDroite = new MAXSwerveModule(7, 8, 90);

  // Le gyroscope
  private Pigeon2 gyro = new Pigeon2(0);

  // Les 3 PID de la base pilotable utilisés par PathPlanner.
  private PPHolonomicDriveController ppHolonomicDriveController = new PPHolonomicDriveController(
      new PIDConstants(PidBasePilotable.kPLineaire, 0, 0),
      new PIDConstants(PidBasePilotable.kPRot, 0, PidBasePilotable.kDRot));

  /*
   * Setpoint generator est une fonction de PathPlanner qui permet de valider
   * si les vitesses demandées en téléop respectent les contraintes mécaniques
   * du robot telles que définies la RobotConfig
   */
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  // Initialisation PoseEstimator
  SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
      Constants.kDriveKinematics,
      Rotation2d.fromDegrees(getAngleGyro()),
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

    // Aller chercher la configuration du robot dans Pathplanner
    // Nécessaire pour swerveSetpointGenerator ET AutoBuilder
    RobotConfig robotConfig = null;
    try {

      robotConfig = RobotConfig.fromGUISettings();

    } catch (Exception e) {
      e.printStackTrace();
    }

    // AutoBuilder permet de générer les trajets autonomes et de followPath
    AutoBuilder.configure(
        this::getPose,
        this::resetOdometry,
        this::getChassisSpeeds,
        (speeds, feedforward) -> conduireChassis(speeds),
        ppHolonomicDriveController,
        robotConfig,
        Constants::isRedAlliance,
        this);

    // Configuration Setpoint Generator
    setpointGenerator = new SwerveSetpointGenerator(
        robotConfig, Units.rotationsToRadians(3.94) // Valeur selon la freespeed du neo 550
    );

    resetSetpoint();
  }

  @Override
  public void periodic() {
    // Update du Pose Estimator
    poseEstimator.update(Rotation2d.fromDegrees(getAngleGyro()), new SwerveModulePosition[] {
        avantGauche.getPosition(),
        avantDroite.getPosition(),
        arriereGauche.getPosition(),
        arriereDroite.getPosition()
    });

    // Update du Field2d
    field2d.setRobotPose(getPose());
    SmartDashboard.putData("Field", field2d);

    setLimelightRobotOrientation();
    addVisionPosition("limelight");
  }

  ///////// FONCTIONS QUI PARLENT DIRECTEMENT AUX MODULES SWERVES
  /// ModuleState = Vitesse de la roue, angle de la roue

  public void setModuleStates(SwerveModuleState[] desiredStates) {
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

  // Sets the wheels into an X formation to prevent movement.
  public void setX() {
    avantGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    avantDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    arriereGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    arriereDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  ///////// FONCTIONS QUI PARLENT DIRECTEMENT AU CHASSIS
  /// ChassisSpeeds = Vx, Vy, Omega

  public ChassisSpeeds getChassisSpeeds() {
    return Constants.kDriveKinematics.toChassisSpeeds(
        avantDroite.getState(), avantGauche.getState(), arriereDroite.getState(), arriereGauche.getState());
  }

  public void conduireChassis(ChassisSpeeds chassisSpeeds) {
    // Ramene la vitesse en intervale de 20 ms
    ChassisSpeeds targetSpeed = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

    SwerveModuleState[] swerveModuleState = Constants.kDriveKinematics.toSwerveModuleStates(targetSpeed);
    setModuleStates(swerveModuleState);
  }

  public void stop() {
    conduireChassis(new ChassisSpeeds(0, 0, 0));
  }

  //////////// FONCTIONS QUI PARLENT AU CHASSIS AVEC LA CORRECTION DU SWERVE
  //////////// SETPOINT GENERATOR
  /// Nécessite swerveSetPointGenerator, donc robotConfig d'un projet PP

  public void conduireChassisSetPoint(ChassisSpeeds speeds) {
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);

    setModuleStates(previousSetpoint.moduleStates());
  }

  public void resetSetpoint() {
    // Nécessaire quand on alterne entre des déplacements autonomes et des
    // déplacements téléop
    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
  }

  ///////// Pose estimator
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) { // pose est à la pose où reset, c'est typiquement l'origine du terrain
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getAngleGyro()),
        new SwerveModulePosition[] {
            avantGauche.getPosition(),
            avantDroite.getPosition(),
            arriereGauche.getPosition(),
            arriereDroite.getPosition()
        },
        pose);
  }

  //////////////////// Limelight
  /// MegaTag2 : il faut connaître l'angle du robot
  /// Donc TOUJOURS ouvrir le robot/compiler en pointant 0°
  /// Voir la documentation limelight, il se peut que ça change l'an prochain !
  public void setLimelightRobotOrientation() {
    LimelightHelpers.SetRobotOrientation(
        "limelight",
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
  }

  public void addVisionPosition(String nomComplet) {

    // parametre limelight
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(nomComplet);
    boolean doRejectUpdate = false;
    if (poseEstimate == null) {
      return;
    }

    if (Math.abs(getOmegaGyro()) > 720) {
      doRejectUpdate = true;
    }
    if (poseEstimate.tagCount == 0) {
      doRejectUpdate = true;
    }
    SmartDashboard.putBoolean(nomComplet, !doRejectUpdate);
    if (!doRejectUpdate) {
      poseEstimator.addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds);
    }
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
  public double getAngleGyro() {
    return gyro.getYaw().getValueAsDouble();
  }

  public double getOmegaGyro() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }



  ///////////// FONCTIONS POUR LE FANCY PATH GENERATOR
  public Command followPath(PathPlannerPath path) {

    return AutoBuilder.followPath(path);
  }

  public void resetPID() {
    ppHolonomicDriveController.reset(getPose(), getChassisSpeeds());
  }

  public void setPID(Pose2d cible) {
    Pose2d current = getPose();
    // Créer une cible PathPlanner et mettre la cible comme pose
    PathPlannerTrajectoryState stateCible = new PathPlannerTrajectoryState();
    stateCible.pose = cible;

    // Nous utilisons le PID que PathPlanner utilise pour ses paths
    ChassisSpeeds chassisSpeeds = ppHolonomicDriveController.calculateRobotRelativeSpeeds(current, stateCible);
    conduireChassis(chassisSpeeds);
  }

}
