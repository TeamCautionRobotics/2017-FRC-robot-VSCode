package org.usfirst.frc.team1492.robot.autonomous.commands;

import org.usfirst.frc.team1492.robot.DriveBase;
import org.usfirst.frc.team1492.robot.autonomous.Command;

public class TurnInPlaceCommand implements Command {

    private DriveBase driveBase;

    private boolean highGear;
    private double speed;
    private double targetAngle;

    private boolean needsToStart;
    private boolean complete;

    private double startAngle;

    public TurnInPlaceCommand(DriveBase driveBase, boolean highGear, double speed, double targetAngle) {
        this.driveBase = driveBase;

        this.highGear = highGear;
        this.speed = speed;
        this.targetAngle = targetAngle;

        reset();
    }

    @Override
    public boolean run() {
        if (needsToStart) {
            driveBase.useHighGear(highGear);
            startAngle = driveBase.getGyroAngle();
            needsToStart = false;
        }

        if (complete) {
            return true;
        } else {
            if (Math.abs(startAngle - driveBase.getGyroAngle()) >= Math.abs(targetAngle)) {
                driveBase.drive(0);
                complete = true;
            } else {
                if (targetAngle > startAngle) {
                    driveBase.drive(-speed, speed);
                } else {
                    driveBase.drive(speed, -speed);
                }
            }
            return false;
        }
    }

    @Override
    public void reset() {
        needsToStart = true;
        complete = false;
    }

}
