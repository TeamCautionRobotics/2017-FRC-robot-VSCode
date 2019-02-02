package org.usfirst.frc.team1492.robot;

import edu.wpi.first.wpilibj.VictorSP;

public class Outfeed {
    private VictorSP outfeed;

    public Outfeed(int motorChannel) {
        outfeed = new VictorSP(motorChannel);
    }

    public void moveOutfeed(double motorPower) {
        outfeed.set(motorPower);
    }

}
