package org.usfirst.frc.team1492.robot.autonomous.commands;

import org.usfirst.frc.team1492.robot.DriveBase;
import org.usfirst.frc.team1492.robot.autonomous.Command;

public class MoveStraightPIDCommand implements Command {

    private DriveBase driveBase;

    private boolean highGear;
    private double distance;

    private boolean needsToStart;
    private boolean complete;

    public MoveStraightPIDCommand(DriveBase driveBase, boolean highGear, double maxSpeed, double distance) {
        this.driveBase = driveBase;

        this.highGear = highGear;
        this.distance = distance;

        driveBase.pidController.setOutputRange(-maxSpeed, maxSpeed);

        reset();
    }

    @Override
    public boolean run() {
        if (needsToStart) {
            driveBase.useHighGear(highGear);
            driveBase.resetEncoders();

            driveBase.pidInit();
            driveBase.pidController.setSetpoint(distance);
            driveBase.pidController.enable();

            needsToStart = false;
        }

        complete = driveBase.pidController.onTarget();
        if (complete) {
            driveBase.pidController.disable();
        }
        return complete;
    }

    @Override
    public void reset() {
        needsToStart = true;
        complete = false;
    }

}
