package frc.robot.displays.alignment;

import java.util.Arrays;

import edu.wpi.first.math.util.Units;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasDisplay;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier.Calculation;
import frc.team_8840_lib.utils.math.units.RectangleBounds;

public class PlacingDisplay extends CanvasDisplay {
    public static final int WIDTH = 320;
    public static final int HEIGHT = 280;
    
    public PlacingDisplay() {
        super("Placing Display", WIDTH, HEIGHT);
    }

    @Override
    public void draw() {
        clearCanvas();

        //Background
        fillStyle(s("#8f8f8f"));
        rect(n(0), n(0), MAX(), MAX(), true);

        //We'll draw a grid to help us visualize the grid.
        //The grid is three deep, and has 9 rows.
        //We'll draw it on the left side of the screen.
        final RectangleBounds[] gridBounds = Measurements.Grid.Rows.getBlueDimensionsList();

        final int[] poleGrids = new int[] {
            0, 2, 3, 5, 6, 8
        };

        final double multValY = 0.85;
        final double multValX = 1.5;

        int i = 0;
        double greatestY = 0;
        for (RectangleBounds bound : gridBounds) {
            fillStyle(
                s(Arrays.binarySearch(poleGrids, (int) i / 3) >= 0 ?
                    "#f5df1b" : "#9e9e9e"
                )
            );

            fillStyle(
                If(
                    nt_value(getCustomBaseNTPath() + "Placing Display/hover"), 
                    CanvasSupplier.IfOperation.EQUAL,
                    n(i),
                    s("#457545")//s("#3bff6f")
                )
            );

            fillStyle(
                If(
                    nt_value(getCustomBaseNTPath() + "Placing Display/selected"), 
                    CanvasSupplier.IfOperation.EQUAL,
                    n(i),
                    s("#3bff6f")
                )
            );

            fillStyle(
                If(
                    nt_value("/SmartDashboard/close"), 
                    CanvasSupplier.IfOperation.EQUAL,
                    n(i),
                    s("#fc0303")
                )
            );

            rect(
                IfElse(
                    nt_value(getCustomBaseNTPath() + "Placing Display/side"), CanvasSupplier.IfOperation.EQUAL, s("red"), 
                    calc(
                        MAX(), Calculation.SUBTRACT, 
                        calc(n(bound.getX() * multValX), Calculation.ADD, n(bound.getWidth() * multValX))
                    ), 
                    n(bound.getX() * multValX)
                ),
                n(bound.getY() * multValY),
                n(bound.getWidth() * multValX),
                n(bound.getHeight() * multValY),
                true
            );

            strokeStyle(
                s("#000000")
            );
            rect(
                IfElse(
                    nt_value(getCustomBaseNTPath() + "Placing Display/side"), CanvasSupplier.IfOperation.EQUAL, s("red"), 
                    calc(
                        MAX(), Calculation.SUBTRACT, 
                        calc(n(bound.getX() * multValX), Calculation.ADD, n(bound.getWidth() * multValX))
                    ), 
                    n(bound.getX() * multValX)
                ),
                n(bound.getY() * multValY),
                n(bound.getWidth() * multValX),
                n(bound.getHeight() * multValY),
                false
            );

            if (greatestY < bound.getY() + bound.getHeight()) {
                greatestY = bound.getY() + bound.getHeight();
            }

            i++;
        }

        double placeLocationsInchesOut = Measurements.Grid.DEPTH + Units.metersToInches(0.5);

        //Show place locations
        for (int g = 0; g < gridBounds.length; g += 3) {
            RectangleBounds bound = gridBounds[g];

            double rawY = bound.getCenterY();

            CanvasSupplier y = n(rawY * multValY);

            CanvasSupplier x = IfElse(
                nt_value(getCustomBaseNTPath() + "Placing Display/side"), CanvasSupplier.IfOperation.EQUAL, s("red"), 
                calc(
                    MAX(), Calculation.SUBTRACT, 
                    n(placeLocationsInchesOut * multValX)
                ), 
                n(placeLocationsInchesOut * multValX)
            );

            fillStyle(
                IfElse(
                    mathf(
                        calc(
                            nt_value(getCustomBaseNTPath() + "Placing Display/selected"),
                            Calculation.DIVIDE,
                            n(3)
                        ), 
                        "floor"
                    ),
                    CanvasSupplier.IfOperation.EQUAL, 
                    n(g / 3),
                    s("#d61ed0"),
                    s("#1f15e6")
                )
            );
            circle(
                x, 
                y, 
                n(5), 
                true
            );
        }

        CanvasSupplier fieldBottomY = n(greatestY * multValY);

        //convert to inches
        CanvasSupplier robotX = nt_value("/SmartDashboard/Field/SwerveRobot[0]").divide(n(0.0254));
        CanvasSupplier robotY = nt_value("/SmartDashboard/Field/SwerveRobot[1]").divide(n(0.0254));

        CanvasSupplier fieldTopY = fieldBottomY.subtract(
            n(Measurements.Field.HEIGHT * multValY)
        );

        CanvasSupplier redSideLeftField = calc(
            MAX(), Calculation.SUBTRACT,
            n(Measurements.Field.WIDTH * multValX)
        );

        rect(
            IfElse(
                nt_value(getCustomBaseNTPath() + "Placing Display/side"), CanvasSupplier.IfOperation.EQUAL, s("red"), 
                redSideLeftField.add(robotX.multiply(n(multValX))),
                robotX.multiply(n(multValX))
            ),
            fieldTopY.add(
                n(Measurements.Field.HEIGHT).subtract(robotY).multiply(n(multValY)) //due to the way Canvas works
            ),
            n(Units.metersToInches(Measurements.Robot.LENGTH) * multValX),
            n(Units.metersToInches(Measurements.Robot.WIDTH) * multValY),
            true
        );
        
    }
}
