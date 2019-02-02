package org.usfirst.frc.team1492.robot;

public enum CameraSettings {
    ORANGE_LIGHT((short) 17, 250),      // Settings for orange light ring - cam_setECV(64017)
    GREEN_COMPETITION((short) 1, 80),   // Settings for green light ring at AZPX 2017 - cam_setECV(20481)
    GREEN_SHOP((short) 2, 150),         // Settings for green light ring in workshop - cam_setECV(38402)
    STATE_CHAMP((short) 0, 80);         // Settings for green light ring at State Championship - cam_setECV(20480)

    final short gain;
    final int compensation;

    private CameraSettings(short gain, int compensation) {
        this.gain = gain;
        this.compensation = compensation;
    }
}
