// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import java.util.Optional;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.IntakeShooter;
import org.carlmontrobotics.commands.Intake;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj.DriverStation;

/** Add your docs here. */
public class AlignToNoteMath extends ProxyCommand {
    static double rednote1X = 13.67;
    static double rednote1Y = 6.99;

    static double rednote2X = 13.67;
    static double rednote2Y = 5.51;

    static double rednote3X = 13.67;
    static double rednote3Y = 4.10;


    static double bluenote1X = 2.90;
    static double bluenote1Y = 6.99;

    static double bluenote2X = 2.90;
    static double bluenote2Y = 5.51;

    static double bluenote3X = 2.90;
    static double bluenote3Y = 4.10;


    static double note4X = 8.28;
    static double note4Y = 7.43;

    static double note5X = 8.28;
    static double note5Y = 5.77;

    static double note6X = 8.28;
    static double note6Y = 4.11;

    static double note7X = 8.28;
    static double note7Y = 2.43;

    static double note8X = 8.28;
    static double note8Y = 0.77;

    static double[] redNoteX = {rednote1X, rednote2X, rednote3X};
    static double[] redNoteY = {rednote1Y, rednote2Y, rednote3Y};

    static double[] blueNoteX = {bluenote1X, bluenote2X, bluenote3X};
    static double[] blueNoteY = {bluenote1Y, bluenote2Y, bluenote3Y};

    static double[] centerNoteX = {note4X, note5X, note6X, note7X, note8X};
    static double[] centerNoteY = {note4Y, note5Y, note6Y, note7Y, note8Y};

    public AlignToNoteMath(Drivetrain dt, IntakeShooter intakeShooter) {

        super(() -> {
            Optional<Alliance> allianceSide = DriverStation.getAlliance();

                double[] redNoteDistances = new double[3];
                double[] blueNoteDistances = new double[3];
                double[] centerNoteDistances = new double[3];


                for (int i = 0; i < redNoteX.length; i++) {
                        redNoteDistances[i] = Math.hypot(
                                        dt.getPose().getX() - redNoteX[i],
                                        dt.getPose().getY() - redNoteY[i]);
                }

                for (int i = 0; i < blueNoteX.length; i++) {
                        blueNoteDistances[i] = Math.hypot(
                                        dt.getPose().getX() - blueNoteX[i],
                                        dt.getPose().getY() - blueNoteY[i]);
                }

                for (int i = 0; i < centerNoteX.length; i++) {
                        centerNoteDistances[i] = Math.hypot(
                                        dt.getPose().getX() - centerNoteX[i],
                                        dt.getPose().getY() - centerNoteY[i]);
                }

                /*
                 * double redNote1distance = Math.hypot(dt.getPose().getX() - rednote1X,
                 * dt.getPose().getY() - rednote1Y); double redNote2distance =
                 * Math.hypot(dt.getPose().getX() - rednote2X, dt.getPose().getY() - rednote2Y);
                 * double redNote3distance = Math.hypot(dt.getPose().getX() - rednote3X,
                 * dt.getPose().getY() - rednote3Y);
                 * 
                 * double blueNote1distance = Math.hypot(dt.getPose().getX() - bluenote1X,
                 * dt.getPose().getY() - bluenote1Y); double blueNote2distance =
                 * Math.hypot(dt.getPose().getX() - bluenote2X, dt.getPose().getY() - bluenote2Y);
                 * double blueNote3distance = Math.hypot(dt.getPose().getX() - bluenote3X,
                 * dt.getPose().getY() - bluenote3Y);
                 * 
                 * double Note4distance = Math.hypot(dt.getPose().getX() - note4X,
                 * dt.getPose().getY() - note4Y); double Note5distance =
                 * Math.hypot(dt.getPose().getX() - note5X, dt.getPose().getY() - note5Y); double
                 * Note6distance = Math.hypot(dt.getPose().getX() - note6X, dt.getPose().getY() -
                 * note6Y); double Note7distance = Math.hypot(dt.getPose().getX() - note7X,
                 * dt.getPose().getY() - note7Y); double Note8distance =
                 * Math.hypot(dt.getPose().getX() - note8X, dt.getPose().getY() - note8Y);
                 */

                // DISTANCES ARRAY
                /*
                 * double [] redNoteDistances = {redNote1distance, redNote2distance,
                 * redNote3distance};
                 * 
                 * double [] blueNoteDistances = {blueNote1distance, blueNote2distance,
                 * blueNote3distance};
                 * 
                 * double [] centerNoteDistances = {Note4distance, Note5distance, Note6distance,
                 * Note7distance, Note8distance};
                 */
                for (int i = 0; i < redNoteDistances.length; i++) {
                        if (redNoteDistances[i] < 2.0) {
                                return new RotateToFieldRelativeAngle(
                                                new Rotation2d(redNoteX[i] + dt
                                                                .getPose()
                                                                .getX(),
                                                                redNoteY[i] - dt.getPose()
                                                                                .getY()),
                                                dt);
                        }
                }



                for (int i = 0; i < blueNoteDistances.length; i++) {
                        if (blueNoteDistances[i] < 2.0) {
                                return new RotateToFieldRelativeAngle(
                                                new Rotation2d(blueNoteX[i] - dt
                                                                .getPose()
                                                                .getX(),
                                                                blueNoteY[i] - dt
                                                                                .getPose()
                                                                                .getY()),
                                                dt);
                        }
                }



                for (int i = 0; i < centerNoteDistances.length; i++) {
                        if (centerNoteDistances[i] < 1.0
                                        && allianceSide.get() == Alliance.Red) {
                                return new RotateToFieldRelativeAngle(
                                                new Rotation2d(centerNoteX[i]
                                                                + dt.getPose().getX(),
                                                                centerNoteY[i] - dt
                                                                                .getPose()
                                                                                .getY()),
                                                dt);
                        } else if (centerNoteDistances[i] < 1.0 && allianceSide
                                        .get() == Alliance.Blue) {
                                return new RotateToFieldRelativeAngle(
                                                new Rotation2d(centerNoteX[i]
                                                                - dt.getPose().getX(),
                                                                centerNoteY[i] - dt
                                                                                .getPose()
                                                                                .getY()),
                                                dt);
                        }
                }
                /*
                 * double redNote1distance1 = Math.abs(redNote1distance); double redNote2distance2 =
                 * Math.abs(redNote2distance); double redNote3distance3 =
                 * Math.abs(redNote3distance);
                 * 
                 * double blueNote1distance1 = Math.abs(blueNote1distance); double
                 * blueNote2distance2 = Math.abs(blueNote2distance); double blueNote3distance3 =
                 * Math.abs(blueNote3distance);
                 * 
                 * double Note4distance4 = Math.abs(Note4distance); double Note5distance5 =
                 * Math.abs(Note5distance); double Note6distance6 = Math.abs(Note6distance); double
                 * Note7distance7 = Math.abs(Note7distance); double Note8distance8 =
                 * Math.abs(Note8distance);
                 */



            // calculate the angle needed to align with note
            // after that, create a command that drives the drivetrain forward
            // once distance sensors detect a note, stop rollers and drivetrain
            // use a timer to stop the drivetrain just in case a note is not intaked
            // TODO: DONE RED
                /*
                 * if (redNote1distance1 < 2.0) { return new RotateToFieldRelativeAngle( new
                 * Rotation2d(dt.getPose().getX(), rednote1Y - dt.getPose().getY()), dt); } if
                 * (redNote2distance2 < 2.0) { return new RotateToFieldRelativeAngle( new
                 * Rotation2d(dt.getPose().getX(), rednote2Y - dt.getPose().getY()), dt); } if
                 * (redNote3distance3 < 2.0) { return new RotateToFieldRelativeAngle( new
                 * Rotation2d(dt.getPose().getX(), rednote3Y - dt.getPose().getY()), dt); } // TODO:
                 * DONE RED
                 * 
                 * 
                 * 
                 * // TODO: DONE BLUE if (blueNote1distance1 < 2.0) { return new
                 * RotateToFieldRelativeAngle( new Rotation2d(bluenote1X - dt.getPose().getX(),
                 * bluenote1Y - dt.getPose().getY()), dt); } if (blueNote2distance2 < 2.0) { return
                 * new RotateToFieldRelativeAngle( new Rotation2d(bluenote2X - dt.getPose().getX(),
                 * bluenote2Y - dt.getPose().getY()), dt); } if (blueNote3distance3 < 2.0) { return
                 * new RotateToFieldRelativeAngle( new Rotation2d(bluenote3X - dt.getPose().getX(),
                 * bluenote3Y - dt.getPose().getY()), dt); } // TODO: DONE BLUE
                 * 
                 * 
                 * 
                 * // TODO: DONE BLUE MIDDLE if (Note4distance4 < 1.0 && allianceSide.get() ==
                 * Alliance.Red) { return new RotateToFieldRelativeAngle(new Rotation2d(
                 * dt.getPose().getX(), note4Y - dt.getPose().getY()), dt); } if (Note5distance5 <
                 * 1.0 && allianceSide.get() == Alliance.Red) { return new
                 * RotateToFieldRelativeAngle(new Rotation2d( dt.getPose().getX(), note5Y -
                 * dt.getPose().getY()), dt); } if (Note6distance6 < 1.0 && allianceSide.get() ==
                 * Alliance.Red) { return new RotateToFieldRelativeAngle(new Rotation2d(
                 * dt.getPose().getX(), note6Y - dt.getPose().getY()), dt); } if (Note7distance7 <
                 * 1.0 && allianceSide.get() == Alliance.Red) { return new
                 * RotateToFieldRelativeAngle(new Rotation2d( dt.getPose().getX(), note7Y -
                 * dt.getPose().getY()), dt); } if (Note8distance8 < 1.0 && allianceSide.get() ==
                 * Alliance.Red) { return new RotateToFieldRelativeAngle(new Rotation2d(
                 * dt.getPose().getX(), note8Y - dt.getPose().getY()), dt); } // TODO: DONE BLUE
                 * MIDDLE
                 * 
                 * 
                 * 
                 * if (Note4distance4 < 1.0 && allianceSide.get() == Alliance.Blue) { return new
                 * RotateToFieldRelativeAngle( new Rotation2d(note4X - dt.getPose().getX(), note4Y -
                 * dt.getPose().getY()), dt); } if (Note5distance5 < 1.0 && allianceSide.get() ==
                 * Alliance.Blue) { return new RotateToFieldRelativeAngle( new Rotation2d(note5X -
                 * dt.getPose().getX(), note5Y - dt.getPose().getY()), dt); } if (Note6distance6 <
                 * 1.0 && allianceSide.get() == Alliance.Blue) { return new
                 * RotateToFieldRelativeAngle( new Rotation2d(note6X - dt.getPose().getX(), note6Y -
                 * dt.getPose().getY()), dt); } if (Note7distance7 < 1.0 && allianceSide.get() ==
                 * Alliance.Blue) { return new RotateToFieldRelativeAngle( new Rotation2d(note7X -
                 * dt.getPose().getX(), note7Y - dt.getPose().getY()), dt); } if (Note8distance8 <
                 * 1.0 && allianceSide.get() == Alliance.Blue) { return new
                 * RotateToFieldRelativeAngle( new Rotation2d(note8X - dt.getPose().getX(), note8Y -
                 * dt.getPose().getY()), dt); }
                 * 
                 * 
                 * //for (){
                 * 
                 * //}
                 * 
                 * // return new RotateToFieldRelativeAngle(new Rotation2d(dt.getPose().getX(), //
                 * blueSpeakerY - dt.getPose().getY()), dt); // get pos and if it is in a something
                 * area, then do some tan^-1 math to align it, then // drive forward
                 * 
                 * /* Optional<Alliance> allianceSide = DriverStation.getAlliance(); if
                 * (allianceSide.get() == Alliance.Red) { // double redAngle =
                 * Math.atan2(redSpeakerY-dt.getPose().getY(), // dt.getPose().getX()); return new
                 * RotateToFieldRelativeAngle( new Rotation2d(dt.getPose().getX(), blueSpeakerY -
                 * dt.getPose().getY()), dt);
                 * 
                 * } else if (allianceSide.get() == Alliance.Blue) { // double blueAngle =
                 * Math.atan2(blueSpeakerY-dt.getPose().getY(), // dt.getPose().getX()); return new
                 * RotateToFieldRelativeAngle( new Rotation2d(redSpeakerX - dt.getPose().getX(),
                 * redSpeakerY - dt.getPose().getY()), dt);
                 * 
                 * } // create an if statement based on alliance side
                 */
            return new InstantCommand();
        });
    }
}
