package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team_8840_lib.utils.controllers.swerve.structs.PIDStruct;

public class ArmSettings {
    public static final double MIN_DIST_OFF_GROUND = Units.inchesToMeters(2);
    
    public static final Translation2d sideViewCenterToBase = new Translation2d(
        Units.inchesToMeters(0),
        Units.inchesToMeters(0)
    );

    public static class Base {
        public static final int PORT = 0;
        public static final boolean INVERTED = false;
        public static final PIDStruct PID = new PIDStruct(0.0, 0.0, 0.0);
        
        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kG = 0.0;
        public static final double kVelocity = 0.0;

        public static double GEAR_RATIO = 1 / 120; //For every 1 rotation of the arm, the motor rotates 120 times.

        public static final double armLengthMeters = Units.inchesToMeters(25);

        //What we'll consider the "0°" position of the arm.
        public static final double LOWER_BOUND_DEGREES = 135;
        //What we'll consider the max position of the arm.
        public static final double UPPER_BOUND_DEGREES = 400;
    }

    public static class Elbow {
        public static final int PORT = 1;
        public static final boolean INVERTED = false;
        public static final PIDStruct PID = new PIDStruct(0.0, 0.0, 0.0);

        public static final double kS = 0.0;
        public static final double kV = 0.0;
        public static final double kG = 0.0;
        public static final double kVelocity = 0.0;

        public static double GEAR_RATIO = 1 / 120; //For every 1 rotation of the arm, the motor rotates 120 times.

        public static final double armLengthMeters = Units.inchesToMeters(31);
    }

    public static class Grabber {
        public static final int PORT = 2;
    }
}
