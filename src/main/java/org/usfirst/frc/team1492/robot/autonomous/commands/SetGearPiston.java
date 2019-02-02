package org.usfirst.frc.team1492.robot.autonomous.commands;

import org.usfirst.frc.team1492.robot.GearPiston;
import org.usfirst.frc.team1492.robot.autonomous.Command;

public class SetGearPiston implements Command {

    private GearPiston gearPiston;

    private boolean out;

    public SetGearPiston(GearPiston gearPiston, boolean out) {
        this.gearPiston = gearPiston;
        this.out = out;
        reset();
    }

    @Override
    public boolean run() {
        gearPiston.latchGear(out);
        return true;
    }

    @Override
    public void reset() {

    }

}
