package org.usfirst.frc.team1492.robot;

import edu.wpi.first.wpilibj.Relay;

public class HumanLoadLight {
    private Relay humanloadlight;

    public enum LightMode {
        GEAR, FUEL, OFF
    }

    public HumanLoadLight(int relayPort) {
        humanloadlight = new Relay(relayPort);
    }

    public void lightOn(LightMode lightMode) {
        switch (lightMode) {
            case GEAR:
                humanloadlight.set(Relay.Value.kForward);
                break;

            case FUEL:
                humanloadlight.set(Relay.Value.kReverse);
                break;

            case OFF:
                humanloadlight.set(Relay.Value.kOn);
                break;
        }
    }
}
