// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;

/** Add your docs here. */
public class FancyPathGeneration {
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> speedSupplier;

    public FancyPathGeneration(Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedSupplier) {
        this.poseSupplier = poseSupplier;
        this.speedSupplier = speedSupplier;
    }

    public PathPlannerPath genererPath(Pose2d cible) {
        PathConstraints constraints = new PathConstraints(3.0, 1.50, Math.toRadians(360), Math.toRadians(360));
        // Hyper Important : Il faut mettre la méthode "flipped" pour ajuster pour
        // RedAlliance
        // Fonction pas mentionnée dans la doc !!
        ChassisSpeeds chassisSpeeds = speedSupplier.get();
        Rotation2d rotation2d = getRotation2d(chassisSpeeds, cible);
        Translation2d translation2d = poseSupplier.get().getTranslation();

        // Point intial comme dans Pathplanner
        Translation2d beginControl = getBeginControl(cible, chassisSpeeds, translation2d);
        Waypoint beginWaypoint = new Waypoint(null, translation2d, beginControl);

        // Point final comme dans Pathplanner
        Translation2d endControl = getEndControl(cible);
        Waypoint endWaypoint = new Waypoint(endControl, cible.getTranslation(), null);

        List<Waypoint> waypoints = List.of(beginWaypoint, endWaypoint);

        // Utilise le déplacement et la vitesse actuelle pour le début du path
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                constraints,
                new IdealStartingState(getVitesseRobot(), rotation2d),
                new GoalEndState(0.0, cible.getRotation()));
        return path;
    }

    public Translation2d getEndControl(Pose2d cible) {
        Rotation2d rotation2d = cible.getRotation();
        Double longueurControle = 0.1;
        Translation2d translation2d = new Translation2d(longueurControle, rotation2d);
        return cible.getTranslation().minus(translation2d);
    }

    public Translation2d getBeginControl(Pose2d cible, ChassisSpeeds chassisSpeeds, Translation2d translation2d) {
        Double coefficientVitesse = 0.75;
        Double vitesseRobot = getVitesseRobot().times(coefficientVitesse).magnitude();
        Rotation2d rotation2d = getRotation2d(chassisSpeeds, cible);
        Translation2d vecteurMouvement = new Translation2d(vitesseRobot, rotation2d);
        return translation2d.plus(vecteurMouvement);
    }

    private Rotation2d getRotation2d(ChassisSpeeds chassisSpeeds, Pose2d cible) {
        Double vitesseMinimum = 0.1;
        if (getVitesseRobot().in(MetersPerSecond) < vitesseMinimum) {
            return getRotationSelonDistance(cible);
        }

        return new Rotation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond);
    }

    private LinearVelocity getVitesseRobot() {
        ChassisSpeeds chassisSpeeds = speedSupplier.get();
        return MetersPerSecond.of(
                new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond).getNorm());
    }

    private Rotation2d getRotationSelonDistance(Pose2d cible) {
        // Mets le robot a 0,0 pour trouver la distance entre le robot et la cible
        Translation2d distance = cible.minus(poseSupplier.get()).getTranslation();
        // Si le robot est proche, on pointe vers la cible ou dans le sens de la cible
        Double distanceMinimum = 0.10;
        return (distance.getNorm() < distanceMinimum) ? cible.getRotation() : distance.getAngle();
    }
}
