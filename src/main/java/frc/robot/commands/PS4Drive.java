package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.BrakeMode;
import frc.robot.utils.ControllerConstants;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.utils.GamePhase;

public class PS4Drive extends CommandBase {
    public static enum DriveMode {
        NORMAL,
        X_BRAKE,
        ZEROED,
        SPINNY_BOI;
        public boolean normalOr(DriveMode driveMode) {
            return this == NORMAL || this == driveMode;
        }
    }


    private PS4Controller controller;
    private DriveSubsystem driveSubsystem;

    private DriveMode driveMode = DriveMode.NORMAL;

    private Trigger xModeTrigger; //Binded to cross button
    private Trigger zeroModeTrigger; //Binded to square button
    private Trigger spinnyBoiTrigger; //Binded to circle button

    public PS4Drive(DriveSubsystem driveSubsystem) {
        this.controller = new PS4Controller(ControllerConstants.DRIVE_PS4_CONTROLLER);
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

        xModeTrigger = new Trigger(controller::getCrossButtonPressed).onTrue(
            Commands.runOnce(() -> {
                if (!driveMode.normalOr(DriveMode.X_BRAKE)) return;
                driveMode = driveMode == DriveMode.NORMAL ? DriveMode.X_BRAKE : DriveMode.NORMAL;
                adjustBrakeModeBasedOnMode();
            })
        );

        zeroModeTrigger = new Trigger(controller::getSquareButton).onTrue(
            Commands.runOnce(() -> {
                if (!driveMode.normalOr(DriveMode.ZEROED)) return;
                driveMode = driveMode == DriveMode.NORMAL ? DriveMode.ZEROED : DriveMode.NORMAL;
            })
        );

        spinnyBoiTrigger = new Trigger(controller::getCircleButton).onTrue(
            Commands.runOnce(() -> {
                if (!driveMode.normalOr(DriveMode.SPINNY_BOI)) return;
                driveMode = driveMode == DriveMode.NORMAL ? DriveMode.SPINNY_BOI : DriveMode.NORMAL;
            })
        );
    }

    public double getForward() {
        return controller.getLeftY();
    }

    public double getStrafe() {
        return controller.getLeftX();
    }

    @Override
    public void initialize() {
        Logger.Log("[" + getName() + "] Initialized.");
    }

    @Override
    public void execute() {
        if (GamePhase.getCurrentPhase() != GamePhase.Teleop) return;
        
        SmartDashboard.putString("Drive Mode", driveMode.name());

        if (driveMode == DriveMode.SPINNY_BOI) {
            driveSubsystem.getSwerveDrive().spinnyBoi(getForward());
            return;
        } else if (driveMode == DriveMode.X_BRAKE) {
            driveSubsystem.getSwerveDrive().applyXBrake();
            driveSubsystem.getSwerveDrive().stop();
            return;
        } else if (driveMode == DriveMode.ZEROED) {
            driveSubsystem.getSwerveDrive().setAllModuleAngles(0);
            driveSubsystem.getSwerveDrive().stop();
            return;
        }
        
        if (Math.abs(getForward()) < 0.1 && Math.abs(getStrafe()) < 0.1) {
            driveSubsystem.getSwerveDrive().stop();
            return;
        }

        Translation2d driveTranslation = new Translation2d(
            getForward(),
            getStrafe()
        );

        driveTranslation = driveTranslation.times(3);

        driveSubsystem.getSwerveDrive().drive(driveTranslation, 0, false, true);
    }

    public void adjustBrakeModeBasedOnMode() {
        if (driveMode == DriveMode.X_BRAKE) {
            driveSubsystem.setBrakeMode(BrakeMode.BRAKE);
        } else {
            driveSubsystem.setBrakeMode(BrakeMode.NORMAL);
        }
    }

    @Override
    public void end(boolean interrupted) {
        Logger.Log("[" + getName() + "] Ended.");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
