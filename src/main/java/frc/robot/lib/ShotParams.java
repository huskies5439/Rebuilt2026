package frc.robot.lib;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;




////Classe pour implémenter une Interpolation pour 3 valeurs Double à la fois.
public class ShotParams implements Interpolatable<ShotParams> {

    private final Double vitesseLanceur;
    private final Double angleHood;
    private final Double vitesseKickeur;

    public ShotParams(Double vitesseLanceur, Double angleHood, Double vitesseKickeur) {
        this.vitesseLanceur = vitesseLanceur;
        this.angleHood = angleHood;
        this.vitesseKickeur = vitesseKickeur;

    }

    //Constructeur alternatif pour créer les ShotsParams à partir d'un CSV
    public ShotParams(String vitesseLanceur, String angleHood, String vitesseKickeur) {
        this(Double.valueOf(vitesseLanceur), Double.valueOf(angleHood), Double.valueOf(vitesseKickeur));
    }

    public Double getVitesseLanceur() {
        return vitesseLanceur;
    }

    public Double getAngleHood() {
        return angleHood;
    }

    public Double getVitesseKickeur() {
        return vitesseKickeur;
    }

    ///Interpoler nos "ShotParams" revient à interpoler sur 3 double en parallèle
    ///L'interpolation pour les doubles est déjà implémentée dans MathUtil.
    @Override
    public ShotParams interpolate(ShotParams endValue, double t) {
        return new ShotParams(
                MathUtil.interpolate(vitesseLanceur, endValue.vitesseLanceur, t),
                MathUtil.interpolate(angleHood, endValue.angleHood, t),
                MathUtil.interpolate(vitesseKickeur, endValue.vitesseKickeur, t));
    }

}
