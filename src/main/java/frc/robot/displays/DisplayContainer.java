package frc.robot.displays;

import frc.robot.displays.alignment.PlacingModule;
import frc.robot.displays.arm.ArmModule;

public class DisplayContainer {
    public static void init() {
        new ArmModule().build();
        new PlacingModule().build();
    }
}
