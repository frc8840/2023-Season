package frc.robot.displays.arm.testing;

import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasDisplay;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier.Calculation;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier.IfOperation;

public class BACHDisplay extends CanvasDisplay {
    public BACHDisplay() {
        super("BACH Display", 200, 200);
    }

    @Override
    public void draw() {
        fillStyle(s("#595959"));
        rect(n(0), n(0), MAX(), MAX(), true);

        //draw a circle
        fillStyle(s("#919191"));

        circle(
            MAX().divide(n(2)), 
            MAX().divide(n(2)), 
            n(80), 
            true
        );

        fillStyle(s("#595959"));

        circle(
            MAX().divide(n(2)), 
            MAX().divide(n(2)), 
            n(70), 
            true
        );

        CanvasSupplier angle = calc(
            n(360),
            Calculation.SUBTRACT,
            nt_value("/SmartDashboard/arm_angle")
        ).divide(n(180)).multiply(n(Math.PI));

        CanvasSupplier angleX = angle.cos().multiply(n(75)).add(MAX().divide(n(2)));
        CanvasSupplier angleY = angle.sin().multiply(n(75)).add(MAX().divide(n(2)));

        fillStyle(s("#FF0000"));

        circle(
            angleX, 
            angleY, 
            n(5), 
            true
        );

        CanvasSupplier chooseAngle = nt_value("/SmartDashboard/choose_angle");

        CanvasSupplier chooseAngleX = chooseAngle.cos().multiply(n(75)).add(MAX().divide(n(2)));
        CanvasSupplier chooseAngleY = chooseAngle.sin().multiply(n(75)).add(MAX().divide(n(2)));

        chooseAngleX = calc(
            chooseAngleX,
            Calculation.ADD,
            IfElse(chooseAngle, IfOperation.EQUAL, angle, n(300), n(0))
        );
        
        fillStyle(s("#00FF00"));

        circle(
            chooseAngleX,
            chooseAngleY, 
            n(5), 
            true
        );
    }
}
