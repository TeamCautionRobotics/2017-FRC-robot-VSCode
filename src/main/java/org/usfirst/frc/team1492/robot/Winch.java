package org.usfirst.frc.team1492.robot;

import edu.wpi.first.wpilibj.VictorSP;

public class Winch {
    private VictorSP winch;

    public Winch(int motorChannel) {
        winch = new VictorSP(motorChannel);
    }

    public void moveWinch(boolean moveUp) {
        if (moveUp) {
            winch.set(-1);
        } else {
            winch.set(0);
        }
    }
}
