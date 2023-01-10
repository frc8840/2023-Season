package frc.robot;

import frc.robot.subsystems.BalanceSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer {
    private final BalanceSubsystem balanceSubsystem;
    private final DriveSubsystem driveSubsystem;

    public RobotContainer() {
        balanceSubsystem = new BalanceSubsystem();
        driveSubsystem = new DriveSubsystem();
    }


    
}
