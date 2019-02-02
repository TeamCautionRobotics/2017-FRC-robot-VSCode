package org.usfirst.frc.team1492.robot.autonomous;

import org.usfirst.frc.team1492.robot.Doors;
import org.usfirst.frc.team1492.robot.DriveBase;
import org.usfirst.frc.team1492.robot.GearPiston;
import org.usfirst.frc.team1492.robot.PixyCamera;
import org.usfirst.frc.team1492.robot.autonomous.commands.AlignWithVisionCommand;
import org.usfirst.frc.team1492.robot.autonomous.commands.DelayCommand;
import org.usfirst.frc.team1492.robot.autonomous.commands.MoveStraightCommand;
import org.usfirst.frc.team1492.robot.autonomous.commands.MoveStraightDistanceCommand;
import org.usfirst.frc.team1492.robot.autonomous.commands.MoveStraightPIDCommand;
import org.usfirst.frc.team1492.robot.autonomous.commands.ResetEncoders;
import org.usfirst.frc.team1492.robot.autonomous.commands.SetGearPiston;
import org.usfirst.frc.team1492.robot.autonomous.commands.TurnInPlaceCommand;
import org.usfirst.frc.team1492.robot.autonomous.commands.TurnToTargetVisionCommand;

public class CommandFactory {
    private DriveBase driveBase;
    private GearPiston gearPiston;
    private Doors doors;
    private PixyCamera pixyCamera;

    public CommandFactory(DriveBase driveBase, GearPiston gearPiston, Doors doors, PixyCamera pixyCamera) {
        this.driveBase = driveBase;
        this.gearPiston = gearPiston;
        this.doors = doors;
        this.pixyCamera = pixyCamera;
    }

    public Command moveStraight(double speed, double time) {
        return moveStraight(false, speed, time);
    }

    public Command moveStraight(boolean highGear, double speed, double time) {
        return moveStraight(highGear, speed, time, false);
    }

    public Command moveStraight(boolean highGear, double speed, double time, boolean keepHeading) {
        return new MoveStraightCommand(driveBase, highGear, speed, time, keepHeading);
    }

    public Command moveStraightDistance(boolean highGear, double speed, double distance,
            boolean stopAtEnd) {
        return new MoveStraightDistanceCommand(driveBase, highGear, speed, distance, stopAtEnd);
    }

    public Command moveStraightPID(double distance) {
        return new MoveStraightPIDCommand(driveBase, false, 1.0, distance);
    }

    public Command moveStraightPID(boolean highGear, double maxSpeed, double distance) {
        return new MoveStraightPIDCommand(driveBase, highGear, maxSpeed, distance);
    }

    public Command turnInPlace(double speed, double targetAngle) {
        return turnInPlace(false, speed, targetAngle);
    }

    public Command turnInPlace(boolean highGear, double speed, double targetAngle) {
        return new TurnInPlaceCommand(driveBase, highGear, speed, targetAngle);
    }

    public Command resetEncoders() {
        return new ResetEncoders(driveBase);
    }

    public Command alignWithVision() {
        return alignWithVision(false);
    }

    public Command alignWithVision(boolean testing) {
        return alignWithVision(testing, false, 0);
    }

    public Command alignWithVision(double encoderStopDistance) {
        return alignWithVision(false, true, encoderStopDistance);
    }

    public Command alignWithVision(boolean testing, boolean encoderStop, double encoderStopDistance) {
        return new AlignWithVisionCommand(driveBase, pixyCamera, testing, encoderStop, encoderStopDistance);
    }

    public Command turnToTarget() {
        return turnToTarget(false);
    }

    public Command turnToTarget(boolean testing) {
        return new TurnToTargetVisionCommand(driveBase, pixyCamera, testing);
    }

    public Command setGearPiston(boolean out) {
        return new SetGearPiston(gearPiston, out);
    }

    public Command delay(double time) {
        return new DelayCommand(time);
    }

}
