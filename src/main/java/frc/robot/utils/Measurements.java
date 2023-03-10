package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.team_8840_lib.utils.math.units.RectangleBounds;

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

    public static class Field {
        //IN INCHES.
        public static final double WIDTH = 54 * 12 + 3.5;
        public static final double HEIGHT = 26 * 12 + 3.5;
    }

    public static class Grid {
        public static final double DEPTH = 54.05;
        public static final double LENGTH = (33 + 18.25 + 47.75 + 18.25 + 47.75 + 18.25 + 33.22);

        public final static Translation2d blueTopLeftPosition = new Translation2d(
            0,
            Measurements.Field.HEIGHT
        );

        public final static Translation2d redTopLeftPosition = new Translation2d(
            Measurements.Field.WIDTH - Measurements.Grid.DEPTH,
            Measurements.Field.HEIGHT - Measurements.Grid.LENGTH
        );

        public static class Rows {
            public static final double topRowWidth = 54.05 - 31.59;
            public static final double middleRowWidth = 31.59 - 14.28;
            public static final double bottomRowWidth = 14.28;

            public static final double lengthOfBottomFieldConeSection = 33;
            public static final double lengthOfTopFieldConeSection = 33.22;
            public static final double lengthOfCubeStation = 18.25;
            public static final double lengthOfInnerConeSection = 47.75;

            public static final RectangleBounds[] getBlueDimensionsList() {
                final double generalTopRowX = 0;
                final double middleRowX = generalTopRowX + Measurements.Grid.Rows.topRowWidth;
                final double bottomRowX = middleRowX + Measurements.Grid.Rows.middleRowWidth;

                final double topRowWidth = Measurements.Grid.Rows.topRowWidth;
                final double middleRowWidth = Measurements.Grid.Rows.middleRowWidth;
                final double bottomRowWidth = Measurements.Grid.Rows.bottomRowWidth;

                final int[] order = new int[] {
                    2, //outer top
                    1, //cube
                    3, //inner outer
                    4, //inner coop
                    1, //cube
                    4, //inner coop
                    3, //inner outer
                    1, //cube
                    5 //outer top
                };

                RectangleBounds[] positions = new RectangleBounds[order.length * 3];

                double currentY = 0;//Measurements.Grid.blueTopLeftPosition.getY();

                for (int i = 0; i < order.length; i++) {
                    double height = 0;

                    switch (order[i]) {
                        case 2:
                            height = Measurements.Grid.Rows.lengthOfTopFieldConeSection;
                            break;
                        case 1:
                            height = Measurements.Grid.Rows.lengthOfCubeStation;
                            break;
                        case 3:
                            height = Measurements.Grid.Rows.lengthOfInnerConeSection;
                            break;
                        case 4:
                            height = Measurements.Grid.Rows.lengthOfInnerConeSection;
                            break;
                        case 5:
                            height = Measurements.Grid.Rows.lengthOfBottomFieldConeSection;
                            break;
                        default:
                            break;
                    }

                    int topIndex = i * 3;
                    int middleIndex = i * 3 + 1;
                    int bottomIndex = i * 3 + 2;

                    positions[topIndex] = new RectangleBounds(
                        generalTopRowX,
                        currentY,
                        topRowWidth,
                        height
                    );

                    positions[middleIndex] = new RectangleBounds(
                        middleRowX,
                        currentY,
                        middleRowWidth,
                        height
                    );

                    positions[bottomIndex] = new RectangleBounds(
                        bottomRowX,
                        currentY,
                        bottomRowWidth,
                        height
                    );

                    currentY = height + currentY;
                }
                
                return positions;
            }

        }
    }
}
