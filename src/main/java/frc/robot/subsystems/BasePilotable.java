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
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
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
   * Setpoint genetator est une fonction de PathPlanner qui permet de valider
   * si les vitesses demandées en téléop respectent les contraintes mécaniques
   * du robot telles que définies la RobotConfig
   */
  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

  PathConstraints pathConstraints;

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

    // Aller chercher la configuration du robot dans Pathplanner
    // Nécessaire pour swerveSetpointGenerator ET AutoBuilder
    RobotConfig robotConfig = null;
    try {

      robotConfig = RobotConfig.fromGUISettings();

    } catch (Exception e) {
      e.printStackTrace();
    }

    pathConstraints = getPPAppSettings();

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
    poseEstimator.update(Rotation2d.fromDegrees(getAngle()), new SwerveModulePosition[] {
        avantGauche.getPosition(),
        avantDroite.getPosition(),
        arriereGauche.getPosition(),
        arriereDroite.getPosition()
    });

    // Update du Field2d
    field2d.setRobotPose(getPose());
    SmartDashboard.putData("Field", field2d);

    setLimelightRobotOrientation();
    addVisionPosition("limelight-haut");
    addVisionPosition("limelight-bas");
  }

  /// ////// MÉTHODE DONNANT DES CONSIGNES À CHAQUE MODULE

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

  /// ///// TÉLÉOP
  public void conduire(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean squared) {

    double deadband = 0.05;
    // appliquer une deadband sur les joysticks et corriger la direction
    xSpeed = -MathUtil.applyDeadband(xSpeed, deadband);
    ySpeed = -MathUtil.applyDeadband(ySpeed, deadband);
    rot = -MathUtil.applyDeadband(rot, deadband);

    if (squared) { // Mettre les joysticks "au carré" pour adoucir les déplacements
      xSpeed = xSpeed * Math.abs(xSpeed);
      ySpeed = ySpeed * Math.abs(ySpeed);
      rot = rot * Math.abs(rot);
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * Constants.maxVitesseLineaire;
    double ySpeedDelivered = ySpeed * Constants.maxVitesseLineaire;
    double rotDelivered = rot * Constants.maxVitesseRotation;

    // inversion du field oriented selon l'alliance
    double invert = 1;
    if (Constants.isRedAlliance()) {
      invert = -1;
    }

    ChassisSpeeds speeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeedDelivered * invert,
            ySpeedDelivered * invert,
            rotDelivered,
            getPose().getRotation())
        : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    // Actualiser le setpoint generator pour corriger le mouvement si nécessaire
    previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);

    setModuleStates(previousSetpoint.moduleStates());
  }

  public void stop() {
    conduire(0, 0, 0, false, false);
  }

  // Sets the wheels into an X formation to prevent movement.
  public void setX() {
    avantGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    avantDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    arriereGauche.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    arriereDroite.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void resetSetpoint() {
    // Caller à chaque fois que l'on retourne à la commande de conduite téléop
    // Sinon les premiers mouvements du robot dépendent de la conduite avant la
    // commande de pathfinding
    previousSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(4));
  }

  /// ////// Pose estimator
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
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

  /// ///////////////// limelight
  /// MegaTag2 : il faut connaître l'angle du robot
  /// Donc TOUJOURS ouvrir le robot/compiler en pointant 0°
  /// Voir la documentation limelight, il se peut que ça change l'an prochain !
  public void setLimelightRobotOrientation() {
    LimelightHelpers.SetRobotOrientation(
        "limelight-haut",
        poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
        0,
        0,
        0,
        0,
        0);
    LimelightHelpers.SetRobotOrientation(
        "limelight-bas",
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

    if (Math.abs(getRate()) > 720) {
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
  public double getAngle() {
    return gyro.getYaw().getValueAsDouble();
  }

  public double getRate() {
    return gyro.getAngularVelocityZWorld().getValueAsDouble();
  }

  public void resetGyro() {
    gyro.setYaw(0);
  }



  /// ///////////// Path Planner
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

  /////////////////////// EN BAS DE CE POINT, CES FONCTIONS NE SONT PEUT-ÊTRE PAS
  /////////////////////// PERTINENTES.
  /// CE SONT DES RESTES DE REEFSCAPE2025 ET DE DEVELOPPMENTAUTOMNE2025 (PID APRÈS
  /////////////////////// FOLLOWPATH)
  /// À VALIDER AU FUR ET À MESURE QUE LES COMMANDES SE PRÉCISENT

  /////////////// On the fly

  public Command followPath(PathPlannerPath path) {

    return AutoBuilder.followPath(path);
    // return AutoBuilder.pathfindToPoseFlipped(cible, constraints);

  }

  public void resetPID() {
    ppHolonomicDriveController.reset(getPose(), getChassisSpeeds());
  }

  public Supplier<Pose2d> getPoseSupplier() {
    return this::getPose;
  }

  public Supplier<ChassisSpeeds> getChassisSpeedsSupplier() {
    return this::getChassisSpeeds;
  }

  ////////////// isProche permet de vérifier la position du robot
  /// Seule fois où on a besoin des Pose2D de l'alliance rouge car on ne passe pas
  ////////////// par PathPlanner
  public boolean isProche(Pose2d cible, double distanceMin) {
    return getPose().getTranslation().getDistance(cible.getTranslation()) < distanceMin;
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

  public void conduireChassisSetPoint(ChassisSpeeds speeds){
     previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, speeds, 0.02);

    setModuleStates(previousSetpoint.moduleStates());
  }

///////Obtenir les paramètres de Pathplanner automatiquement
  public PathConstraints getPPAppSettings() {
    PathPlannerPath pathConfig = null;

    try {
      pathConfig = PathPlannerPath.fromPathFile("config");
    } catch (Exception e) {
      e.printStackTrace();
    }

    return pathConfig.getGlobalConstraints();
  }

  public double getPPMaxVitesseLineaire(){
    return getPPAppSettings().maxVelocityMPS();
  }

  public double getPPMaxAccelerationLineaire(){
    return getPPAppSettings().maxAccelerationMPSSq();
  }

  public double getPPMaxVitesseAngulaire() {
    return Math.toDegrees(getPPAppSettings().maxAngularVelocityRadPerSec());
  }

  public double getPPMaxAccelerationAngulaire() {
    return Math.toDegrees(getPPAppSettings().maxAngularAccelerationRadPerSecSq());
  }


}
