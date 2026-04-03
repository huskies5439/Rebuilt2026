// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileReader;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.List;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.opencsv.CSVReader;
import com.opencsv.CSVReaderBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
	private double omegaRobot = 0;

	////// Pour obtenir les valeurs de la Base Pilotable
	private final Supplier<Pose2d> poseSupplier;
	private final Supplier<ChassisSpeeds> speedSupplier;
	private final DoubleSupplier omegaSupplier;

	private double trimLanceur;
	private double trimKickeur;
	private double trimTourelle;

	private final double deltaLanceur;
	private final double deltaKickeur;
	private final double deltaTourelle;

	// Look-Up-Table séparée des paramètres de tir afin d'itérer plus rapidement
	InterpolatingDoubleTreeMap lutTOF = new InterpolatingDoubleTreeMap();

	// Look-Up-Table custom (voir lib) pour interpoler sur 3 doubles à la fois
	// Il faut spécifier une fonction d'interpolation inverse pour trouver où on se
	// trouve entre deux distances.
	// C'est déjà implémenté pour les doubles.
	InterpolatingTreeMap<Double, ShotParams> lutShotParams = new InterpolatingTreeMap<>(
			InverseInterpolator.forDouble(),
			ShotParams::interpolate);

	Field2d field2d = new Field2d();

	private boolean lancerActif = false;

	public Superstructure(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedSupplier,
			DoubleSupplier omegaSupplier) {
		this.poseSupplier = poseSupplier;
		this.speedSupplier = speedSupplier;
		this.omegaSupplier = omegaSupplier;

		trimLanceur = 0.0;
		trimKickeur = 0.0;
		trimTourelle = 0.0;

		deltaLanceur = 1.0;
		deltaKickeur = 1.0;
		deltaTourelle = 2.0;

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
		omegaRobot = Math.toRadians(omegaSupplier.getAsDouble());

		// Mise à jour de la cibleRéelle et de la cible virtuelle
		setCibleReelle();
		calculCibleVirtuelle();

		field2d.setRobotPose(poseRobot);
		field2d.getObject("cible virtuelle").setPose(new Pose2d(cibleVirtuelle, Rotation2d.kZero));
		SmartDashboard.putData("field2d", field2d);
		SmartDashboard.putString("Shift Time",
				String.format("%.2f", new BigDecimal(remainingTime()).setScale(2, RoundingMode.FLOOR)));
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

	public boolean cibleIsHub() {
		return cibleReelle == Cible.hubRouge || cibleReelle == Cible.hubBleu;
	}

	public void calculCibleVirtuelle() {
		double lastDistance = Double.MAX_VALUE;
		Translation2d cibleTemporaire = cibleReelle;

		for (int i = 0; i < 10; i++) {
			double distance = getDistanceCible(cibleTemporaire);
			double timeOfFlight = getTOF(distance);

			ChassisSpeeds tourrelleSpeeds = getVitesseTourelle();
			double vxTourelle = tourrelleSpeeds.vxMetersPerSecond;
			double vyTourelle = tourrelleSpeeds.vyMetersPerSecond;

			Translation2d projectionFutur = new Translation2d(vxTourelle * timeOfFlight, vyTourelle * timeOfFlight);

			cibleTemporaire = cibleReelle.minus(projectionFutur);

			if (Math.abs(distance - lastDistance) <= 0.1) {
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

	public double getAngleCible(Translation2d cible) {///// C'est l'angle par rapport à l'origine de la tourelle.
		Rotation2d angleVecteurCible = getVecteurCible(cible).getAngle();
		Rotation2d angleTourelle = poseRobot.getRotation().rotateBy(Rotation2d.k180deg);// Tourelle pointe par en
																						// arrière
		return angleVecteurCible.minus(angleTourelle).getDegrees();
	}

	/////// Vecteur avec la cible virtuelle calculée par itération
	public Translation2d getVecteurCibleVirtuelle() {
		return getVecteurCible(cibleVirtuelle);
	}

	public double getDistanceCibleVirtuelle() {
		return getDistanceCible(cibleVirtuelle);
	}

	public double getAngleCibleVirtuelle() {
		return getAngleCible(cibleVirtuelle);
	}

	//////// Vecteur avec la cible réelle (lancer statique)
	public Translation2d getVecteurCibleReelle() {
		return getVecteurCible(cibleReelle);
	}

	public double getDistanceCibleReelle() {
		return getDistanceCible(cibleReelle);
	}

	public double getAngleCibleReelle() {
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

	public boolean isProcheRectangle(Translation2d cible, double demiX, double demiY) {

		Pose2d pose = poseRobot.plus(deplacementTourelle);

		double poseX = pose.getX();
		double poseY = pose.getY();

		return poseX < cible.getX() + demiX
				&& poseX > cible.getX() - demiX
				&& poseY < cible.getY() + demiY
				&& poseY > cible.getY() - demiY;

	}

	public boolean isProcheTrench() {
		// double rayon = 1.0;
		double demiX = 0.35;
		double demiY = 0.61;
		return isProcheRectangle(PoseTrench.trenchBleuDepot, demiX, demiY) ||
				isProcheRectangle(PoseTrench.trenchBleuOutpost, demiX, demiY) ||
				isProcheRectangle(PoseTrench.trenchRougeDepot, demiX, demiY) ||
				isProcheRectangle(PoseTrench.trenchRougeOutpost, demiX, demiY);
	}

	public boolean getLancerActif() {
		return this.lancerActif;
	}

	public void setLancerActif(boolean lancerActif) {
		if (DriverStation.isTeleop()) {
			this.lancerActif = lancerActif;
		} else {
			this.lancerActif = false;
		}

	}

	/////// Cinématique d'un point P (tourelle) sur un corps rigide (robot)
	/// voir l'équation de la vitesse:
	/////// https://courses.grainger.illinois.edu/tam212/su2025/rkg.html#rkg-er

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
		double vitesseX = -deplacementTourelleRotation.getY() * omegaRobot;
		double vitesseY = deplacementTourelleRotation.getX() * omegaRobot;
		return new ChassisSpeeds(vitesseX, vitesseY, 0);
	}

	// Correspond au calcul final de \vect{v}_{Q}
	public ChassisSpeeds getVitesseTourelle() {
		ChassisSpeeds composanteRotationTourelle = getComposanteRotationTourelle();
		return ChassisSpeeds.fromRobotRelativeSpeeds(chassisSpeedsRobot, poseRobot.getRotation())// \vect{v}_{P}
				.plus(composanteRotationTourelle);
	}

	public boolean isHubActive(int decalage) {
		Optional<Alliance> alliance = DriverStation.getAlliance();

		// Hub is always enabled in autonomous.
		if (DriverStation.isAutonomousEnabled()) {
			return true;
		}
		// At this point, if we're not teleop enabled, there is no hub.
		if (!DriverStation.isTeleopEnabled()) {
			return false;
		}

		// We're teleop enabled, compute.
		double matchTime = DriverStation.getMatchTime();
		String gameData = DriverStation.getGameSpecificMessage();
		// If we have no game data, we cannot compute, assume hub is active, as its
		// likely early in teleop.
		if (gameData.isEmpty()) {
			return true;
		}
		boolean redInactiveFirst = false;
		switch (gameData.charAt(0)) {
			case 'R' -> redInactiveFirst = true;
			case 'B' -> redInactiveFirst = false;
			default -> {
				// If we have invalid game data, assume hub is active.
				return true;
			}
		}

		// Shift was is active for blue if red won auto, or red if blue won auto.
		boolean shift1Active = switch (alliance.get()) {
			case Red -> !redInactiveFirst;
			case Blue -> redInactiveFirst;
		};

		if (matchTime > 130 + decalage) {
			// Transition shift, hub is active.
			return true;
		} else if (matchTime > 105 + decalage) {
			// Shift 1
			return shift1Active;
		} else if (matchTime > 80 + decalage) {
			// Shift 2
			return !shift1Active;
		} else if (matchTime > 55 + decalage) {
			// Shift 3
			return shift1Active;
		} else if (matchTime > 30 + decalage) {
			// Shift 4
			return !shift1Active;
		} else {
			// End game, hub always active.
			return true;
		}
	}

	public void isHubActive() {
		isHubActive(0);
	}

	public double remainingTime() {
		double matchTime = DriverStation.getMatchTime();

		if (matchTime > 130) {
			// Transition shift, hub is active.
			return matchTime - 130;
		} else if (matchTime > 105) {
			// Shift 1
			return matchTime - 105;
		} else if (matchTime > 80) {
			// Shift 2
			return matchTime - 80;
		} else if (matchTime > 55) {
			// Shift 3
			return matchTime - 55;
		} else if (matchTime > 30) {
			// Shift 4
			return matchTime - 30;
		} else {
			// End game, hub always active.
			return matchTime;
		}
	}

	public boolean hasWinAuto() {
		if(DriverStation.isEnabled()){
			String gameData = DriverStation.getGameSpecificMessage();
			switch (gameData.charAt(0)) {
				case 'R':
					return Constants.isRedAlliance() ? true : false;
				case 'B':
					return Constants.isRedAlliance() ? false : true;
				default :
					return false;
			}
		}else{
			return false; 
		}
	}

	// TRIM

	public double getTrimLanceur() {
		return trimLanceur;
	}

	public double getTrimKickeur() {
		return trimKickeur;
	}

	public double getTrimTourelle() {
		return trimTourelle;
	}

	public Command plusTrimTourelle(){
		return Commands.runOnce(()-> {trimTourelle += deltaTourelle;});
	}

	public Command moinsTrimTourelle(){
		return Commands.runOnce(()-> {trimTourelle -= deltaTourelle;});
	}

	public Command plusTrimLanceur(){
		return Commands.runOnce(()-> {trimLanceur += deltaLanceur;});
	}

	public Command moinsTrimLanceur(){
		return Commands.runOnce(()-> {trimLanceur -= deltaLanceur;});
	}

	public Command plusTrimKickeur(){
		return Commands.runOnce(()-> {trimKickeur += deltaKickeur;});
	}

	public Command moinsTrimKickeur(){
		return Commands.runOnce(()-> {trimKickeur -= deltaKickeur;});
	}
}