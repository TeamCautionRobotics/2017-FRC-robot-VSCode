package org.usfirst.frc.team1492.robot;

import java.util.ArrayList;
import java.util.TimerTask;

public enum PixyCamera {
    INSTANCE;

    private CameraSettings pixyCameraSettings;
    private boolean pixyOpen = false;

    private PixyCamera() {
        // This would normally be in a static block, but is here because enum reasons.
        System.loadLibrary("pixy_java");

        initPixy();
    }

    public void initPixy() {
        initPixy(false);
    }

    public void initPixy(boolean retry) {
        if (pixyOpen) {
            System.err.format("[PixyCamera] initPixy: pixy is open, closing.");
            closePixy();
        }

        int status = pixy.pixy_init();
        if (status < 0) {
            System.err.format("[PixyCamera] initPixy: pixy_init returned %d%n", status);
            pixyOpen = false;

            if (retry) {
                System.err.format("[PixyCamera] initPixy: retry true, retrying at %d.%n",
                        System.currentTimeMillis() + 50);
                // Try again in 50 milliseconds
                new java.util.Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        initPixy(true); // Keep retrying
                    }
                }, 50);
            }
        } else {
            System.err.format("[PixyCamera] initPixy: pixy_init successful at %d.%n", System.currentTimeMillis());

            pixy.pixy_cam_set_auto_exposure_compensation((short) 0);
            pixy.pixy_cam_set_auto_white_balance((short) 0);
    
            // cam_setWBV(0x884040)
            pixy.pixy_cam_set_white_balance_value((short) 64, (short) 64, (short) 136);
    
            setPixySettings(pixyCameraSettings);
            pixyOpen = true;
        }
    }

    public void closePixy() {
        pixy.pixy_close();
        pixyOpen = false;
    }

    public void setPixySettings(CameraSettings cameraSettings) {
        if (cameraSettings != null) {
            pixy.pixy_cam_set_exposure_compensation(cameraSettings.gain, cameraSettings.compensation);
            pixyCameraSettings = cameraSettings;
        }
    }

    public boolean blocksAreNew() {
        if (pixyOpen) {
            int status = pixy.pixy_cam_get_auto_exposure_compensation();

            // Pixy is probably unavailable
            if (status < 0) {
                closePixy();
                System.err.format(
                        "[PixyCamera] blocksAreNew: getAEC status %d, at %d, reiniting Pixy.%n",
                        status, System.currentTimeMillis());

                // Make sure initPixy is called in 20 milliseconds
                new java.util.Timer().schedule(new TimerTask() {
                    @Override
                    public void run() {
                        initPixy(true); // Keep retrying if Pixy unavailable
                    }
                }, 20);
                return false;
            } else {
                // Only check for blocks if Pixy is confirmed to be connected
                return pixy.pixy_blocks_are_new() == 1;
            }
        }
        return false;
    }

    public int getBlocks(int maxCount, BlockArray blocks) {
        return pixy.pixy_get_blocks(maxCount, blocks);
    }

    public static Block[] pickBlocks(BlockArray blocks, int blockCount) {
        if (blockCount < 2) {
            return null;
        } else if (blockCount == 2) {
            return new Block[] {blocks.getitem(0), blocks.getitem(1)};
        } else if (blockCount > 2) {
            System.out.println("[pixy-vision] blocks over 2");

            ArrayList<Block> targets = new ArrayList<>();
            for (int i = 0; i < blockCount; i++) {
                targets.add(blocks.getitem(i));
            }

            targets.sort((Block o1, Block o2) -> Integer.compare(o1.getX(), o2.getX()));

            System.out.println("[pixy-vision]");
            for (int i = 0; i < targets.size(); i++) {
                System.out.format(" target %d, x is %d", i, targets.get(i).getX());
            }
            System.out.println("");
            return new Block[] {targets.get(0), targets.get(targets.size() - 1)};
        }
        return null;
    }
}
