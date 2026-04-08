// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BasePilotable;
import frc.robot.subsystems.Carroussel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Tourelle;

public class TournerCarroussel extends Command {

    Lanceur lanceur;
    Hood hood;
    Tourelle tourelle;
    BasePilotable basePilotable;
    Carroussel carroussel;
    Kickeur kickeur;
    Superstructure superstructure;

    boolean conditionSansTourelle;
    boolean convoyer;
    double toleranceTourelle;

    public TournerCarroussel(
        Lanceur lanceur,
        Hood hood,
        Tourelle tourelle,
        BasePilotable basePilotable,
        Carroussel carroussel,
        Kickeur kickeur,
        Superstructure superstructure
    ) {
        this.lanceur = lanceur;
        this.hood = hood;
        this.tourelle = tourelle;
        this.basePilotable = basePilotable;
        this.carroussel = carroussel;
        this.kickeur = kickeur;
        this.superstructure = superstructure;
        addRequirements(carroussel);
    }

    @Override
    public void initialize() {
        conditionSansTourelle = false;
        convoyer = false;

    }

    @Override
    public void execute() {
        if (superstructure.cibleIsHub()) {
            toleranceTourelle = 10.0;
        }
        else {
            toleranceTourelle = 30.0;
        }

        if (lanceur.atCible() && hood.atCible() && kickeur.atCible() && tourelle.atCible(toleranceTourelle)) {
            convoyer = true;
        }

        if (convoyer) {
            carroussel.tourner();
        }
        else {
            carroussel.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        carroussel.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
