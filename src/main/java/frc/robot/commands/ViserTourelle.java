// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Tourelle;

public class ViserTourelle extends Command {

    Superstructure superstructure;
    Tourelle tourelle;
    double deltaAngleAbsolu;

    double angleActuelAbsolu;
    double angleCibleAbsolu;

    double angleCibleReel;

    double limiteFil = 195;

    public ViserTourelle(Tourelle tourelle, Superstructure superstructure) {
        this.tourelle = tourelle;

        this.superstructure = superstructure;

        addRequirements(tourelle);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        angleCibleAbsolu = superstructure.getAngleCibleVirtuelle() + superstructure.getTrimTourelle();
        angleActuelAbsolu = tourelle.getAngleAbsolu();

        deltaAngleAbsolu = angleCibleAbsolu - angleActuelAbsolu;
        if (Math.abs(deltaAngleAbsolu) > 180.0) {

            angleCibleReel = tourelle.getAngleReel() + angleComplementaire360(deltaAngleAbsolu);

        }
        else {
            angleCibleReel = tourelle.getAngleReel() + deltaAngleAbsolu;
        }
        if (Math.abs(angleCibleReel) < limiteFil) {
            tourelle.setPID(angleCibleReel);
        }
        else {
            grandTour();
        }
    }

    private void grandTour() {
        angleCibleReel = angleActuelAbsolu + deltaAngleAbsolu;

        if (!(Math.abs(angleCibleReel) < limiteFil)) {
            angleCibleReel = angleComplementaire360(angleCibleReel);
        }

        tourelle.setPID(angleCibleReel);
    }

    private double angleComplementaire360(double angleLong) {
        return (Math.abs(angleLong) - 360.0) * (Math.signum(angleLong));
    }

    @Override
    public void end(boolean interrupted) {
        tourelle.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
