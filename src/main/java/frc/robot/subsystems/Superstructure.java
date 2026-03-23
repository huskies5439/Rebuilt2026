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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Cible;
import frc.robot.Constants.PoseTrench;
import frc.robot.lib.ShotParams;

@Logged
public class Superstructure extends SubsystemBase {

	Transform2d deplacementTourelle = new Transform2d(-0.153, -0.153, Rotation2d.kZero);

	private Translation2d cibleReelle = new Translation2d();
	private Translation2d cibleVirtuelle = new Translation2d(); // Ajustée en fonction de la vitesse du robot
	private Pose2d poseRobot = new Pose2d();
	private ChassisSpeeds chassisSpeedsRobot = new ChassisSpeeds();

	////// Pour obtenir les valeurs de la Base Pilotable
	private final Supplier<Pose2d> poseSupplier;
	private final Supplier<ChassisSpeeds> speedSupplier;

	// Look-Up-Table séparée des paramètres de tir afin d'itérer plus rapidement
	InterpolatingDoubleTreeMap lutTOF = new InterpolatingDoubleTreeMap();

	// Look-Up-Table custom (voir lib) pour interpoler sur 3 doubles à la fois
	// Il faut spécifier une fonction d'interpolation inverse pour trouver où on se
	// trouve entre deux distances.
	// C'est déjà implémenté pour les doubles.
	InterpolatingTreeMap<Double, ShotParams> lutShotParams = new InterpolatingTreeMap<>(
			InverseInterpolator.forDouble(),
			ShotParams::interpolate);

	public Superstructure(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier) {
		this.poseSupplier = poseSupplier;
		this.speedSupplier = speedSupplier;

		// Création de la LUT
		// Il faut ajouter la librairie OpenCSV dans le build.gradle
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
	}

	@Override
	public void periodic() {
		// Mise à jour des valeurs critiques de contrôle
		poseRobot = poseSupplier.get();
		chassisSpeedsRobot = speedSupplier.get();

		// Mise à jour de la cibleRéelle et de la cible virtuelle
		setCibleReelle();
		calculCibleVirtuelle();
	}

	public void setCibleReelle() {
		if (Constants.isRedAlliance()) {
			if (poseRobot.getX() > Cible.hubRouge.getX()) {
				cibleReelle = Cible.hubRouge;
			} else if (poseRobot.getY() > Cible.hubRouge.getY()) {
				cibleReelle = Cible.souffleuseOutPostRouge;
			} else {
				cibleReelle = Cible.souffleuseDepotRouge;
			}
		} else {
			if (poseRobot.getX() < Cible.hubBleu.getX()) {
				cibleReelle = Cible.hubBleu;
			} else if (poseRobot.getY() > Cible.hubBleu.getY()) {
				cibleReelle = Cible.souffleuseDepotBleu;
			} else {
				cibleReelle = Cible.souffleuseOutPostBleu;
			}
		}
	}

	public void calculCibleVirtuelle() {
		double lastDistance = Double.MAX_VALUE;
		Translation2d cibleTemporaire = cibleReelle;

		for (int i = 0; i < 5; i++) {
			double distance = getDistanceCible(cibleTemporaire);
			double timeOfFlight = getTOF(distance);

			ChassisSpeeds tourrelleSpeeds = getVitesseTourelle();
			double vxTourelle = tourrelleSpeeds.vxMetersPerSecond;
			double vyTourelle = tourrelleSpeeds.vyMetersPerSecond;

			Translation2d projectionFutur = new Translation2d(vxTourelle * timeOfFlight, vyTourelle * timeOfFlight);

			cibleTemporaire = cibleReelle.minus(projectionFutur);

			if (Math.abs(distance - lastDistance) <= 0.25) {
				break;
			}
			lastDistance = distance;
		}

		cibleVirtuelle = cibleTemporaire;
	}

	/////////// Vecteur entre la tourelle et une cible générique
	public Translation2d getVecteurCible(Translation2d cible) {
		Translation2d poseTourelle = poseRobot.plus(deplacementTourelle).getTranslation();

		return cible.minus(poseTourelle);
	}

	public double getDistanceCible(Translation2d cible) {
		return getVecteurCible(cible).getNorm();
	}

	public Rotation2d getAngleCible(Translation2d cible) {///// C'est l'angle par rapport à l'origine de la tourelle.
		Rotation2d angleVecteurCible = getVecteurCible(cible).getAngle();
		Rotation2d angleTourelle = poseRobot.getRotation().rotateBy(Rotation2d.k180deg);// Tourelle pointe par en
																						// arrière
		return angleVecteurCible.minus(angleTourelle);
	}

	/////// Vecteur avec la cible virtuelle calculée par itération
	public Translation2d getVecteurCibleVirtuelle() {
		return getVecteurCible(cibleVirtuelle);
	}

	public double getDistanceCibleVirtuelle() {
		return getDistanceCible(cibleVirtuelle);
	}

	public Rotation2d getAngleCibleVirtuelle() {
		return getAngleCible(cibleVirtuelle);
	}

	//////// Vecteur avec la cible réelle (lancer statique)
	public Translation2d getVecteurCibleReelle() {
		return getVecteurCible(cibleReelle);
	}

	public double getDistanceCibleReelle() {
		return getDistanceCible(cibleReelle);
	}

	public Rotation2d getAngleCibleReelle() {
		return getAngleCible(cibleReelle);
	}

	/////// LOOK-UP-TABLE
	public ShotParams getGeneriqueShotParams(double distance) {
		return lutShotParams.get(distance);
	}

	public ShotParams getShotParams(boolean dynamique) {
		if (dynamique) {
			return getGeneriqueShotParams(getDistanceCibleVirtuelle());
		} else {
			return getGeneriqueShotParams(getDistanceCibleReelle());
		}

	}

	public double getTOF(double distance) {
		return lutTOF.get(distance);
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

	/////// Cinématique d'un point P (tourelle) sur un corps rigide (robot)
	/// voir l'équation de la vitesse:
	/////// https://courses.grainger.illinois.edu/tam212/su2025/rkg.htmll#rkg-er

	// Correspond à \vect{r}_{PQ}
	// C'est le vecteur de déplacement du point Q (centre du robot) vers le point P
	// (tourelle),
	// rotationné avec le corps rigide (robot)
	public Translation2d getDeplacementTourelleAvecRotation() {
		return deplacementTourelle.getTranslation().rotateBy(poseRobot.getRotation());
	}

	// Correspond à \vect{\omega} \cross \vect{r}_{PQ}
	public ChassisSpeeds getComposanteRotationTourelle() {
		Translation2d deplacementTourelleRotation = getDeplacementTourelleAvecRotation();
		//// Formule du produit vectoriel obtenu avec la méthode du déterminant
		double vitesseX = -deplacementTourelleRotation.getY() * chassisSpeedsRobot.omegaRadiansPerSecond;
		double vitesseY = deplacementTourelleRotation.getX() * chassisSpeedsRobot.omegaRadiansPerSecond;
		return new ChassisSpeeds(vitesseX, vitesseY, 0);
	}

	// Correspond au calcul final de \vect{v}_{Q}
	public ChassisSpeeds getVitesseTourelle() {
		ChassisSpeeds composanteRotationTourelle = getComposanteRotationTourelle();
		return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsRobot, poseRobot.getRotation())// \vect{v}_{P}
				.plus(composanteRotationTourelle);
	}

	// public double normeVecteurLancer() {
	// double vitesseBallon;
	// if (getDistanceCibleTourelle() < 1) {
	// vitesseBallon = Constants.RegimeLanceur.vitesseProche;
	// } else {
	// vitesseBallon = Constants.RegimeLanceur.vitesseLoin;
	// }
	// // Ajouter la correction de la vitesse ici sur la vitesse Ballonn

	// return vitesseBallon;
	// }

	// public double conversionBallonRouleau() {
	// // Norme vecteur donne la vitesse voulue du BALLON. Il faut convertir pour
	// // obtenir la vitesse correspondante du rouleau du lanceur
	// return normeVecteurLancer() / (Units.inchesToMeters(4) *
	// Constants.coefficientFrictionBallon);
	// }

	// // pitch
	// public double pitchVecteurLancer() { // ajouter un clamp pour éviter de
	// briser le hood
	// double vitesseBallon = normeVecteurLancer();
	// double distance = getDistanceCibleTourelle();
	// double hauteur = cible.getZ();
	// return Math.toDegrees(
	// Math.atan(
	// (Math.pow(vitesseBallon, 2)
	// + Math.sqrt(
	// Math.pow(vitesseBallon, 4)
	// - Math.pow(Constants.g, 2) * Math.pow(distance, 2)
	// - 2 * Constants.g * Math.pow(vitesseBallon, 2) * hauteur))
	// / Constants.g * distance));
	// }

	// // Yaw
	// public double yawVecteurLancer() {
	// return getAngleCible().getDegrees(); // ajouter la correction pour la vitesse
	// ici
	// }

}
