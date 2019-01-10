package org.firstinspires.ftc.teamcode;

public enum _GoldPlacement {
    // The fields indicate placement of contours in the processed image. The image will be split into three portions.
    LEFT,
    CENTER,
    RIGHT,
    UNKNOWN;


    @Override
    public String toString() {
        switch(this) {
            case LEFT:
                return "Left";
            case CENTER:
                return "Center";
            case RIGHT:
                return "Right";
            case UNKNOWN:
                return "Unknown";
            default:
                return "Unknown";
        }
    }

}
