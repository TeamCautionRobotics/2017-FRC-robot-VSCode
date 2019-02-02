package org.usfirst.frc.team1492.robot.autonomous.commands;

import org.usfirst.frc.team1492.robot.DriveBase;
import org.usfirst.frc.team1492.robot.autonomous.Command;

public class ResetEncoders implements Command {

    private DriveBase driveBase;

    public ResetEncoders(DriveBase driveBase) {
        this.driveBase = driveBase;
    }

    @Override
    public boolean run() {
        driveBase.resetEncoders();
        return true;
    }

    @Override
    public void reset() {

    }

}
