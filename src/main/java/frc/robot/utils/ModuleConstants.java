package frc.robot.utils;

public class ModuleConstants {
    public static class TopLeft {
        public static final int DRIVE = 11;
        public static final int TURN = 12;
        public static final int ENCODER = 23;
        public static final double OFFSET = 105.8203; //106.4355;

        public static final double INVERT_OFFSET = 74.17;
    }

    public static class TopRight {
        public static final int DRIVE = 18;
        public static final int TURN = 17;
        public static final int ENCODER = 22;
        public static final double OFFSET = 323.877; //144.4043;

        public static final double INVERT_OFFSET = 35.51;
    }

    public static class BottomRight {
        public static final int DRIVE = 15;
        public static final int TURN = 16;
        public static final int ENCODER = 21;
        public static final double OFFSET = 41.8359; //42.7148;

        public static final double INVERT_OFFSET = 139.31;
    }

    public static class BottomLeft {
        public static final int DRIVE = 13;
        public static final int TURN = 14;
        public static final int ENCODER = 24;
        public static final double OFFSET = 215.332; //35.6836;

        public static final double INVERT_OFFSET = 144.14;
    }

    public static final int PIGEON_ID = 42;
}
