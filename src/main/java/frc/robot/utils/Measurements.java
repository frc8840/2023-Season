package frc.robot.utils;

import edu.wpi.first.math.util.Units;

//All measurements must be in meters.
public class Measurements {
    public static class Robot {
        public static final double WIDTH = Units.inchesToMeters(24);
        public static final double LENGTH = Units.inchesToMeters(30);
        public static final double BASE_HEIGHT = Units.inchesToMeters(6);
    }

    public static class Arm {
        public static final double HEIGHT_FROM_GROUND = Units.inchesToMeters(23.224);
    }

    public static class DockingStation {
        /**
         * Representation of docking station top:
         *  ________________
         * |                | 
         * |                | 4ft
         * |                | (width)
         * |________________|
         *     8ft (length)
         * 
         * note: drawings are not to scale
         */
        public static final double WIDTH = Units.feetToMeters(4);
        public static final double LENGTH = Units.feetToMeters(8);
    }
}
