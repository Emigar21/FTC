package org.firstinspires.ftc.teamcode;

public class Constants {
    static class DistanceConstants {
        public static final double OMNI_CIRC_CM = (4 * 2.54) * Math.PI;
        public static final double PULLEY_CIRC_CM = 11.83752;
        public static final double MAX_ARM_EXTENSION_CM = 20;
        public static final double TILE_LENGTH_CM = 60.96;
        public static final int LOW_RUNG_ELEV_TICKS = -5;
    }
    static class EncoderConstants {
        public static final int TRQNADO_TPR = 1440;
        public static final double YELLOW_JKT_TPR = 537.6;
        public static final int HD_HEX_TPR= 1120;
        public static final int CORE_HEX_TPR= 288;
        public static final int CLOSED_CLAW_TICKS = (int) (EncoderConstants.CORE_HEX_TPR * .4);
        public static final int OPEN_CLAW_TICKS = 10;
    }

    static class ActuatorConstants { // Changed too also include motor constants, like claw limit
        public static final double SCISSOR_CLAW_CLOSED = 0;
        public static final double SCISSOR_CLAW_OPEN = 1;
        public static final double SCISSOR_CLAW_UNFOLDED = 1;
        public static final double SCISSOR_CLAW_FOLDED = 0;
        public static final int ELBOW_LOW_TICKS = 10;
        public static final int ELBOW_MID_TICKS = (int) (EncoderConstants.CORE_HEX_TPR * .15) * 3;
        public static final int ELBOW_HIGH_TICKS = (int) (EncoderConstants.CORE_HEX_TPR * .2 * 3);
    }

    public class SpeedConstants {
        public static final double CHAIN_EXTRUDE_SPEED = 1;
        public static final double CHAIN_RETRACT_SPEED = -1;
    }
}
