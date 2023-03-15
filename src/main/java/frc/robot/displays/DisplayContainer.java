package frc.robot.displays;

import frc.robot.displays.alignment.PlacingModule;
import frc.robot.displays.arm.ArmModule;
import frc.robot.displays.arm.testing.BACHModule;
import frc.robot.displays.driving.DriveModeModule;

public class DisplayContainer {
    public static void init() {
        new ArmModule().build();
        new PlacingModule().build();
        new BACHModule().build();
        new DriveModeModule().build();
    }
}
