package frc.robot.utils;

import edu.wpi.first.math.util.Units;

//All measurements must be in meters.
public class Measurements {
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
