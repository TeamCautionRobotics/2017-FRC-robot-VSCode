package org.usfirst.frc.team1492.robot.autonomous.commands;

import org.usfirst.frc.team1492.robot.Block;
import org.usfirst.frc.team1492.robot.BlockArray;
import org.usfirst.frc.team1492.robot.DriveBase;
import org.usfirst.frc.team1492.robot.PixyCamera;
import org.usfirst.frc.team1492.robot.pixy;
import org.usfirst.frc.team1492.robot.autonomous.Command;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlignWithVisionCommand implements Command {

    private DriveBase driveBase;
    private BlockArray blocks = new BlockArray(100);
    private PixyCamera pixyCamera;
    private boolean locked = false;
    private boolean aimed = false;
    private int overCount = 0;
    private Preferences preferences;
    private boolean testing;
    private double stopY;
    private boolean encoderStop;
    private double encoderStopDistance;
    private boolean initialized = false;


    public AlignWithVisionCommand(DriveBase driveBase, PixyCamera pixyCamera, boolean testing,
            boolean encoderStop, double encoderStopDistance) {
        this.testing = testing;
        this.driveBase = driveBase;
        this.pixyCamera = pixyCamera;
        this.encoderStop = encoderStop;
        this.encoderStopDistance = encoderStopDistance;

        preferences = Preferences.getInstance();
        // preferences.putDouble("vision/stopY", 108);
        // preferences.putDouble("vision/base", 0.7);
        // preferences.putDouble("vision/gain", 0.7);
        // preferences.putDouble("vision/xOffset", 0.02);
        // preferences.putDouble("vision/theta", 25);

        updateTrackingMove("constructed");
    }

    @Override
    public boolean run() {
        if (!initialized) {
            if (encoderStop) {
                driveBase.resetEncoders();
            }
            initialized = true;
        }
        if (pixyCamera.blocksAreNew()) {
            int count = pixyCamera.getBlocks(100, blocks);

            SmartDashboard.putNumber("Number blocks", count);

            stopY = preferences.getDouble("vision/stopY", 155);

            if (count > 2) {
                overCount++;
                SmartDashboard.putNumber("Number blocks while over", count);
                SmartDashboard.putNumber("over count", overCount);
            }

            Block[] targets = PixyCamera.pickBlocks(blocks, count);

            if (targets != null) {
                if (!locked && !aimed) {
                    updateTrackingMove("locked");
                    locked = true;
                }

                // Block[] targets = {blocks.getitem(0), blocks.getitem(1)};

                showBlocks(targets);

                double centerX =
                        ((targets[0].getX() + targets[1].getX()) / 2.0) / (double) pixy.PIXY_MAX_X;
                double width = ((targets[0].getWidth() + targets[1].getWidth()) / 2.0)
                        / (double) pixy.PIXY_MAX_X;
                double yAvg = (targets[0].getY() + targets[1].getY()) / 2.0;

                if ((!encoderStop && yAvg > stopY)
                        || (encoderStop && driveBase.getDistance() >= encoderStopDistance)) {
                    driveBase.drive(0);
                    aimed = true;
                    updateTrackingMove("Aimed!!!");
                } else {
                    double offset = preferences.getDouble("vision/xOffset", 0.0);
                    double trim = ((centerX + offset) * 2) - 1;
                    SmartDashboard.putNumber("trim value", trim);

                    double baseSpeed = preferences.getDouble("vision/base", 0.5);
                    double gain = preferences.getDouble("vision/gain", 0.6);

                    double leftSpeed = baseSpeed + (gain * trim);
                    double rightSpeed = baseSpeed - (gain * trim);

                    if (!testing) {
                        driveBase.drive(leftSpeed, rightSpeed);
                    }
                }

            } else {
                // When using encoders, make sure to stop if target is lost
                if (encoderStop && driveBase.getDistance() >= encoderStopDistance) {
                    driveBase.drive(0);
                    aimed = true;
                    updateTrackingMove("Aimed no-target!!!");
                }
                if (!aimed && locked) {
                    // keep driving
                } else {
                    driveBase.drive(0);
                }
            }
        }
        return testing ? false : aimed;
    }

    @Override
    public void reset() {
        locked = false;
        aimed = false;
        overCount = 0;
        updateTrackingMove("reset");
        initialized = false;
    }

    private void showBlocks(Block[] blocks) {
        double centerX = ((blocks[0].getX() + blocks[1].getX()) / 2.0) / (double) pixy.PIXY_MAX_X;
        double width = ((blocks[0].getWidth() + blocks[1].getWidth()) / 2.0);
        double height = (blocks[0].getHeight() + blocks[1].getHeight()) / 2.0;

        SmartDashboard.putNumber("Calculated distance", getDistance(height));
        double theta = Math.toRadians(preferences.getDouble("vision/theta", 25));
        SmartDashboard.putNumber("Trig distance", (5.0 * 200) / (2.0 * height * Math.tan(theta)));

        double angle = getAngle(blocks);
        SmartDashboard.putNumber("angle", angle);
        double sidedAngle = sideAngle(angle, blocks);
        SmartDashboard.putNumber("sided angle", sidedAngle);

        SmartDashboard.putNumber("Y avg", (blocks[0].getY() + blocks[1].getY()) / 2.0);

        SmartDashboard.putNumber("Center X", centerX);
        SmartDashboard.putNumber("Width avg frac", width / (double) pixy.PIXY_MAX_X);
        SmartDashboard.putNumber("target width", blocks[0].getX() - blocks[1].getX());

        SmartDashboard.putNumber("Width ratio", width / (blocks[0].getX() - blocks[1].getX()));
        SmartDashboard.putNumber("Height ratio",
                (double) blocks[0].getHeight() / blocks[1].getHeight());
        SmartDashboard.putNumber("Y delta", ((double) blocks[0].getY() - blocks[1].getY()));
        SmartDashboard.putNumber("Aspect Ratio average", width / height);

        SmartDashboard.putNumber("Width avg", (blocks[0].getWidth() + blocks[1].getWidth()) / 2.0);
        SmartDashboard.putNumber("Height avg", height);

        SmartDashboard.putNumber("Height 0", blocks[0].getHeight());
        SmartDashboard.putNumber("Width 0", blocks[0].getWidth());
        SmartDashboard.putNumber("Y 0", blocks[0].getY());
        SmartDashboard.putNumber("X 0", blocks[0].getX());
        SmartDashboard.putNumber("Aspect ratio 0",
                (double) blocks[0].getWidth() / (double) blocks[0].getHeight());

        SmartDashboard.putNumber("Height 1", blocks[1].getHeight());
        SmartDashboard.putNumber("Width 1", blocks[1].getWidth());
        SmartDashboard.putNumber("Y 1", blocks[1].getY());
        SmartDashboard.putNumber("X 1", blocks[1].getX());
        SmartDashboard.putNumber("Aspect ratio 1",
                (double) blocks[1].getWidth() / (double) blocks[1].getHeight());

        SmartDashboard.putNumber("H delta", (blocks[0].getHeight() - blocks[1].getHeight()));
        SmartDashboard.putNumber("W delta", (blocks[0].getWidth() - blocks[1].getWidth()));
    }

    private void updateTrackingMove(String move) {
        SmartDashboard.putString("Tracking movement", move);
    }

    private double getDistance(double height) {
        return Math.exp(-0.035 * height) * 107.0;
    }

    private double getAngle(Block[] blocks) {
        double widthOfScreen = 320;
        double FOV = Math.toRadians(75);
        double realDistanceApart = 8.25;

        double separation = Math.max(blocks[0].getX(), blocks[1].getX())
                - Math.min(blocks[0].getX(), blocks[1].getX());
        double distance = getDistance((blocks[0].getHeight() + blocks[1].getHeight()) / 2.0);

        double ratio = (separation * 2 * distance * Math.tan(FOV / 2))
                / (widthOfScreen * realDistanceApart);
        SmartDashboard.putNumber("ratio", ratio);
        double angle = Math.acos(Math.min(ratio, 1));

        return angle;
    }

    private double sideAngle(double angle, Block[] blocks) {
        double yMinX = blocks[0].getY() < blocks[1].getY() ? blocks[0].getX() : blocks[1].getX();
        double yMaxX = blocks[0].getY() > blocks[1].getY() ? blocks[0].getX() : blocks[1].getX();

        return yMinX > yMaxX ? 90 + angle : 90 - angle;
    }
}
