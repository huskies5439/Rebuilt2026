// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileReader;
import java.util.List;
import java.util.function.Supplier;

import com.opencsv.CSVReader;
import com.opencsv.CSVReaderBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Cible;
import frc.robot.Constants.PoseTrench;
import frc.robot.lib.ShotParams;

@Logged
public class Superstructure extends SubsystemBase {

  Transform2d deplacementTourelle = new Transform2d(-0.153, -0.153, Rotation2d.kZero);

  private Translation3d cible = new Translation3d();
  private Pose2d poseRobot = new Pose2d();
  private ChassisSpeeds chassisSpeedsRobot = new ChassisSpeeds();

  ////// Pour obtenir les valeurs de la Base Pilotable
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedSupplier;

  InterpolatingDoubleTreeMap lutTOF = new InterpolatingDoubleTreeMap();

  InterpolatingTreeMap<Double, ShotParams> lutShotParams = new InterpolatingTreeMap<>(
      InverseInterpolator.forDouble(),
      ShotParams::interpolate);

  public Superstructure(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedSupplier = speedSupplier;
  }

  @Override
  public void periodic() {

    try {
      FileReader fileReader = new FileReader(new File(Filesystem.getDeployDirectory(), "shooter.csv"));

      CSVReader csvReader = new CSVReaderBuilder(fileReader)
          .withSkipLines(1)
          .build();

      List<String[]> allData = csvReader.readAll();
      for (String[] row : allData) {
        lutTOF.put(Double.valueOf(row[0]), Double.valueOf(row[1]));

        lutShotParams.put(Double.valueOf(row[0]), new ShotParams(row[2], row[3], row[4]));

      }

    } catch (Exception e) {
      e.printStackTrace();
    }

    // Mise à jour des valeurs critiques de contrôle
    poseRobot = poseSupplier.get();
    chassisSpeedsRobot = speedSupplier.get();

    setCible();

  }

  public void setCible() {
    if (Constants.isRedAlliance()) {
      if (poseRobot.getX() > Cible.hubRouge.getX()) {
        cible = Cible.hubRouge;
      } else if (poseRobot.getY() > Cible.hubRouge.getY()) {
        cible = Cible.souffleuseOutPostRouge;
      } else {
        cible = Cible.souffleuseDepotRouge;
      }
    } else {
      if (poseRobot.getX() < Cible.hubBleu.getX()) {
        cible = Cible.hubBleu;
      } else if (poseRobot.getY() > Cible.hubBleu.getY()) {
        cible = Cible.souffleuseDepotBleu;
      } else {
        cible = Cible.souffleuseOutPostBleu;
      }
    }
  }

  public Translation2d getVecteurCibleTourelle() {
    Translation2d poseTourelle = poseRobot.plus(deplacementTourelle).getTranslation();
    return cible.toTranslation2d().minus(poseTourelle);
  }

  public double getDistanceCibleTourelle() {
    return getVecteurCibleTourelle().getNorm();
  }

  public Rotation2d getAngleCible() {
    Rotation2d angleVecteurCible = getVecteurCibleTourelle().getAngle();
    Rotation2d angleTourelle = poseRobot.getRotation().rotateBy(Rotation2d.k180deg);
    return angleVecteurCible.minus(angleTourelle);
  }

  public double normeVecteurLancer() {
    double vitesseBallon;
    if (getDistanceCibleTourelle() < 1) {
      vitesseBallon = Constants.RegimeLanceur.vitesseProche;
    } else {
      vitesseBallon = Constants.RegimeLanceur.vitesseLoin;
    }
    // Ajouter la correction de la vitesse ici sur la vitesse Ballonn

    return vitesseBallon;
  }

  public double conversionBallonRouleau() {
    // Norme vecteur donne la vitesse voulue du BALLON. Il faut convertir pour
    // obtenir la vitesse correspondante du rouleau du lanceur
    return normeVecteurLancer() / (Units.inchesToMeters(4) * Constants.coefficientFrictionBallon);
  }

  // pitch
  public double pitchVecteurLancer() { // ajouter un clamp pour éviter de briser le hood
    double vitesseBallon = normeVecteurLancer();
    double distance = getDistanceCibleTourelle();
    double hauteur = cible.getZ();
    return Math.toDegrees(
        Math.atan(
            (Math.pow(vitesseBallon, 2)
                + Math.sqrt(
                    Math.pow(vitesseBallon, 4)
                        - Math.pow(Constants.g, 2) * Math.pow(distance, 2)
                        - 2 * Constants.g * Math.pow(vitesseBallon, 2) * hauteur))
                / Constants.g * distance));
  }

  // Yaw
  public double yawVecteurLancer() {
    return getAngleCible().getDegrees(); // ajouter la correction pour la vitesse ici
  }

  //////// Protection Tourelle Trench

  public boolean isProche(Translation2d cible, double rayon, boolean checkTourelle) {

    Pose2d pose = poseRobot;

    if (checkTourelle) {
      pose = poseRobot.plus(deplacementTourelle);
    }

    return cible.minus(pose.getTranslation()).getNorm() < rayon;

  }

  public boolean isProche(Pose2d cible, double rayon, boolean checkTourelle) {
    return isProche(cible.getTranslation(), rayon, checkTourelle);
  }

  public boolean isProche(Pose2d cible, double rayon) {// Si on ne spécifie pas de checkTourelle, on veut le centre du
                                                       // robot !
    return isProche(cible.getTranslation(), rayon, false);
  }

  public boolean isProcheTrench() {
    double rayon = 1.0;
    return isProche(PoseTrench.trenchBleuDepot, rayon, true) ||
        isProche(PoseTrench.trenchBleuOutpost, rayon, true) ||
        isProche(PoseTrench.trenchRougeDepot, rayon, true) ||
        isProche(PoseTrench.trenchRougeOutpost, rayon, true);
  }

  /// calculs compliqués

  public Translation2d getDeplacementTourelleRotation() {
    return deplacementTourelle.getTranslation().rotateBy(poseRobot.getRotation());
  }

  public ChassisSpeeds getComposanteRotationTourelle() {
    Translation2d deplacementTourelleRotation = getDeplacementTourelleRotation();
    double vitesseX = -deplacementTourelleRotation.getY() * chassisSpeedsRobot.omegaRadiansPerSecond;
    double vitesseY = deplacementTourelleRotation.getX() * chassisSpeedsRobot.omegaRadiansPerSecond;
    return new ChassisSpeeds(vitesseX, vitesseY, 0);
  }

  public ChassisSpeeds getVitesseTourelle() {
    ChassisSpeeds composanteRotationTourelle = getComposanteRotationTourelle();
    return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsRobot, poseRobot.getRotation())
        .plus(composanteRotationTourelle);
  }

  public ShotParams getLancer(double distance) {
    return lutShotParams.get(distance);
  }

  public double getTOF(double distance) {
    return lutTOF.get(distance);
  }

  public double calculAjustementVitesse(double distance, double timeOfFlight) {
    double vitesseTourelle = 2; 
    return distance + vitesseTourelle * timeOfFlight;
  }

  public double calculRecursif(double distance) {
    double timeOfFlight = 0.0;
    double lastDistance = 0.0;

    for (int i = 0; i < 5; i++) {
      timeOfFlight = getTOF(distance);
      distance = calculAjustementVitesse(distance, timeOfFlight);

      if (Math.abs(distance - lastDistance) <= 0.05) {
        break;
      }
      lastDistance = distance;
    }

    return distance;
  }

}
