package frc.robot.displays;

import frc.robot.displays.alignment.PlacingModule;
import frc.robot.displays.arm.ArmModule;
import frc.robot.displays.arm.testing.BACHModule;

public class DisplayContainer {
    public static void init() {
        new ArmModule().build();
        new PlacingModule().build();
        new BACHModule().build();
    }
}
