// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.ShotParams;
import frc.robot.subsystems.Kickeur;
import frc.robot.subsystems.Lanceur;
import frc.robot.subsystems.Superstructure;

public class PreLancerAutonome extends Command {

    ShotParams shotParams;

    Superstructure superstructure;
    Kickeur kickeur;
    Lanceur lanceur;

    public PreLancerAutonome(Superstructure superstructure, Kickeur kickeur, Lanceur lanceur) {
        this.superstructure = superstructure;
        this.kickeur = kickeur;
        this.lanceur = lanceur;

        addRequirements(lanceur, kickeur, superstructure);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        shotParams = superstructure.getGeneriqueShotParams(3.25);

        lanceur.setPID(shotParams.getVitesseLanceur());
        kickeur.setPID(shotParams.getVitesseKickeur());

    }

    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
