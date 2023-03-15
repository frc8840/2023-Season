package frc.robot.displays.driving;

import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasDisplay;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier;
import frc.team_8840_lib.input.communication.dashboard.components.canvas.CanvasSupplier.IfOperation;

public class DriverModeDisplay extends CanvasDisplay {
    public static final int WIDTH = 100;
    public static final int HEIGHT = 100;
    
    public DriverModeDisplay() {
        super("DriveModeDisplay", WIDTH, HEIGHT);
    }

    @Override
    public void draw() {
        CanvasSupplier drive_mode = nt_value("/SmartDashboard/Drive Mode");
        
        clearCanvas();

        fillStyle(
            IfElse(
                drive_mode, IfOperation.EQUAL, s("NORMAL"), 
                s("#FFFFFF"),
                IfElse(
                    drive_mode, IfOperation.EQUAL, s("X_BRAKE"),
                    s("#707070"),
                    IfElse(
                        drive_mode, IfOperation.EQUAL, s("ZEROED"),
                        s("#ff0000"),
                        IfElse(
                            drive_mode, IfOperation.EQUAL, s("SPINNY_BOI"),
                            s("#15ff00"),
                            IfElse(
                                drive_mode, IfOperation.EQUAL, s("TESTING"),
                                s("#fff700"),
                                s("#000000")
                            )
                        )
                    )
                )
            )
        );

        rect(n(0), n(0), MAX(), MAX(), true);
    }
}
