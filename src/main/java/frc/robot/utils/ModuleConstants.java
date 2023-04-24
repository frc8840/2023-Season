package frc.robot.utils;

public class ModuleConstants {
    //All constants. The offsets are set so that when the drive is set to a positive value, the motors are facing forward.
    public static class TopLeft {
        public static final int DRIVE = 11;
        public static final int TURN = 12;
        public static final int ENCODER = 23;
        public static final double OFFSET = 105.8203; //106.4355;
    }

    public static class TopRight {
        public static final int DRIVE = 18;
        public static final int TURN = 17;
        public static final int ENCODER = 22;
        public static final double OFFSET = 323.877; //144.4043;
    }

    public static class BottomRight {
        public static final int DRIVE = 16;
        public static final int TURN = 15;
        public static final int ENCODER = 21;
        public static final double OFFSET = 41.8359; //42.7148; //171.3 + 49 - 180
    }

    public static class BottomLeft {
        public static final int DRIVE = 13;
        public static final int TURN = 14;
        public static final int ENCODER = 24;
        public static final double OFFSET = 215.332; //35.6836;
    }

    public static final int PIGEON_ID = 42;
}
