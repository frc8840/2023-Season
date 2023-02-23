package frc.robot.displays.arm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.ArmSettings;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasDisplay;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier.Calculation;

public class ArmDisplay extends CanvasDisplay {
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;
    
    public ArmDisplay() {
        super("Arm Display", WIDTH, HEIGHT);
    }

    @Override
    public void draw() {
        clearCanvas();
        
        //Background
        fillStyle(s("#8f8f8f"));
        rect(n(0), n(0), MAX(), MAX(), true);

        //Background #2
        fillStyle(s("#666666"));
        rect(
            n(10), n(10), 
            MAX().subtract(n(20)),
            MAX().subtract(n(20)),
            true
        );

        final double robot_length = Units.metersToInches(Measurements.Robot.LENGTH) * 3.5;

        final double ROBOT_HEIGHT = Units.metersToInches(Measurements.Robot.BASE_HEIGHT) * 3.5 + 10;

        final double ARM_HEIGHT_ABOVE_GROUND = Units.metersToInches(Measurements.Arm.HEIGHT_FROM_GROUND) * 3.5 + 10;

        /*
         * We'll then draw the base of the robot.
         * This is just a simple box so we can see where the arm is relative to the robot.
         */
        fillStyle(s("#000000"));
        rect(
            MAX().divide(n(2)).subtract(n(robot_length / 2)),
            MAX().subtract(n(ROBOT_HEIGHT)),
            n(robot_length),
            n(ROBOT_HEIGHT - 10),
            true
        );

        /**
         * We'll then grab the arm lengths from the ArmSettings class.
         */
        final double baseLength = Units.metersToInches(ArmSettings.Base.armLengthMeters) * 3.5;
        final double elbowLength = Units.metersToInches(ArmSettings.Elbow.armLengthMeters) * 3.5;

        /**
         * We'll create two canvas suppliers to represent the location of the base of the arm.
         */
        CanvasSupplier baseX = MAX().divide(n(2));
        CanvasSupplier baseY = MAX().subtract(n(ARM_HEIGHT_ABOVE_GROUND));

        //We'll draw two lines from the robot corners to the arm base, to represent what the mechanism looks like
        lineWidth(n(2));
        strokeStyle(s("#e8e8e8"));

        line(
            MAX().divide(n(2)).subtract(n(robot_length / 2)),
            MAX().subtract(n(ROBOT_HEIGHT)),
            baseX,
            baseY
        );

        line(
            MAX().divide(n(2)).add(n(robot_length / 2)),
            MAX().subtract(n(ROBOT_HEIGHT)),
            baseX,
            baseY
        );

        /**
         * We'll also create two NetworkTable values to recieve the angle of the base and elbow.
         */
        CanvasSupplier angle = nt_value(
            CommunicationManager.base() + "/arm/baseAngle"
        ).add(n(ArmSettings.Base.LOWER_BOUND_DEGREES));

        CanvasSupplier elbowAngle = nt_value(CommunicationManager.base() + "/arm/realElbowAngle");

        /**
         * Since the angles from NetworkTables are in degrees, we'll convert them to radians.
         */
        angle = angle.divide(n(180)).multiply(n(Math.PI));
        elbowAngle = elbowAngle.divide(n(180)).multiply(n(Math.PI));

        /**
         * We'll then use the angle to calculate the x and y values of where the elbow should be.
         */
        CanvasSupplier xElbowBase = angle.cos().multiply(n(baseLength));
        CanvasSupplier yElbowBase = angle.sin().multiply(n(baseLength));

        /**
         * We'll then add the x and y values of the base to the x and y values of the elbow.
         */
        xElbowBase = xElbowBase.add(calc(MAX(), Calculation.DIVIDE, n(2)));
        yElbowBase = yElbowBase.add(calc(MAX(), Calculation.SUBTRACT, n(ARM_HEIGHT_ABOVE_GROUND)));

        //Draw a line from the base to the elbow
        lineWidth(n(2));
        strokeStyle(s("#ff1b0a"));
        
        line(baseX, baseY, xElbowBase, yElbowBase);

        /**
         * We then need to find the end of the elbow.
         */
        CanvasSupplier xElbowWrist = elbowAngle.cos().multiply(n(elbowLength));
        CanvasSupplier yElbowWrist = elbowAngle.sin().multiply(n(elbowLength));

        xElbowWrist = xElbowWrist.add(xElbowBase);
        yElbowWrist = yElbowWrist.add(yElbowBase);

        //Draw a line from the elbow to the wrist
        strokeStyle(s("#07fc03"));

        line(xElbowBase, yElbowBase, xElbowWrist, yElbowWrist);

        //Reset the line width and color
        lineWidth(n(1));
        strokeStyle(s("#000000"));

        /**
         * We'll finish it up by drawing some small rectangles at the base, elbow, and wrist.
         * This is useful for debugging, and generally seeing where the arm is.
         */
        final double rectSize = 10;

        //Draw a small rectangle at the base
        fillStyle(s("#002aff"));
        rect(
            baseX.subtract(n(rectSize / 2)),
            baseY.subtract(n(rectSize / 2)),
            n(rectSize),
            n(rectSize),
            true
        );

        //Draw a small rectangle at the elbow
        fillStyle(s("#ff1b0a"));
        rect(
            xElbowBase.subtract(n(rectSize / 2)),
            yElbowBase.subtract(n(rectSize / 2)),
            n(rectSize),
            n(rectSize),
            true
        );

        //Draw a small rectangle at the wrist
        fillStyle(s("#07fc03"));
        rect(
            xElbowWrist.subtract(n(rectSize / 2)),
            yElbowWrist.subtract(n(rectSize / 2)),
            n(rectSize),
            n(rectSize),
            true
        );

        //Draw a circle at the mouse
        fillStyle(s("#b0b0b0"));
        circle(
            mouseX(),
            mouseY(), 
            n(5), 
            true
        );
    }
}
