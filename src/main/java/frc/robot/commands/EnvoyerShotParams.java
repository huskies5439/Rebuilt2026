// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.ShotParams;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;

public class EnvoyerShotParams extends Command {

    Lanceur lanceur;
    Hood hood;
    Kickeur kickeur;
    Superstructure superstructure;

    ShotParams shotParams;

    boolean dynamique;
    boolean avecFin; //Si on veut régler la séquence de fin avec un autre commande


    public EnvoyerShotParams(Lanceur lanceur, Hood hood, Kickeur kickeur, Superstructure superstructure, boolean avecFin) {

        this.lanceur = lanceur;
        this.hood = hood;
        this.kickeur = kickeur;
        this.superstructure = superstructure;
        this.avecFin = avecFin;

        addRequirements(lanceur, hood, kickeur, superstructure);

    }

    @Override
    public void initialize() {
        dynamique = true;
        
        lanceur.resetPID();
        kickeur.resetPID();
        hood.resetPID();


    }

    @Override
    public void execute() {
        shotParams = superstructure.getShotParams(dynamique);
        SmartDashboard.putNumber("shot params - lanceur", shotParams.getVitesseLanceur());
        SmartDashboard.putNumber("shot params - kickeur", shotParams.getVitesseKickeur());
        SmartDashboard.putNumber("shot params - Hood", shotParams.getAngleHood());
        lanceur.setPID(shotParams.getVitesseLanceur() + superstructure.getTrimLanceur());
        hood.setPID(shotParams.getAngleHood());
        kickeur.setPID(shotParams.getVitesseKickeur() + superstructure.getTrimKickeur());
    }

    @Override
    public void end(boolean interrupted) {
        if (avecFin) {
            lanceur.stop();
            hood.stop();
            kickeur.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
