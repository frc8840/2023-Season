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
import frc.team_8840_lib.input.controls.SimulatedController;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.GamePhase;
import frc.team_8840_lib.utils.controls.Axis;

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

    private final static double maxSpeed = 3;

    private PS4Controller controller;
    private SimulatedController simulatedController;

    private DriveSubsystem driveSubsystem;

    private DriveMode driveMode = DriveMode.NORMAL;

    private Trigger xModeTrigger; //Binded to cross button
    private Trigger zeroModeTrigger; //Binded to square button
    private Trigger spinnyBoiTrigger; //Binded to circle button

    private double rotation = 0;

    public PS4Drive(DriveSubsystem driveSubsystem) {
        //Setup Controllers
        this.controller = new PS4Controller(ControllerConstants.DRIVE_PS4_CONTROLLER);
        this.simulatedController = new SimulatedController();

        //Setup Drive Subsystem
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

        if (Robot.isReal()) {
            //Add Triggers to Controllers
            xModeTrigger = new Trigger(controller::getCrossButtonPressed).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.X_BRAKE)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.X_BRAKE : DriveMode.NORMAL;
                    adjustBrakeModeBasedOnMode();

                    if (driveMode == DriveMode.X_BRAKE) {
                        setRumble(0.2, 0.2);
                    }
                })
            );

            zeroModeTrigger = new Trigger(controller::getSquareButton).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.ZEROED)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.ZEROED : DriveMode.NORMAL;

                    if (driveMode == DriveMode.ZEROED) {
                        setRumble(0.2, 0.2);
                    }
                })
            );

            spinnyBoiTrigger = new Trigger(controller::getCircleButton).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.SPINNY_BOI)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.SPINNY_BOI : DriveMode.NORMAL;

                    if (driveMode == DriveMode.SPINNY_BOI) {
                        setRumble(0.2, 0.2);
                    }
                })
            );
        }
    }

    public double getForward() {
        return Robot.isSimulation() ? simulatedController.getAxis(Axis.Vertical) : -controller.getLeftY();
    }

    public double getStrafe() {
        return Robot.isSimulation() ? simulatedController.getAxis(Axis.Horizontal) : controller.getLeftX();
    }

    public double getRightX() {
        return Robot.isSimulation() ? Math.cos(simulatedController.getAxis(Axis.Rotation)) : controller.getRightX();
    }

    public double getRightY() {
        return Robot.isSimulation() ? Math.sin(simulatedController.getAxis(Axis.Vertical)) : controller.getRightY();
    }

    public void setRumble(double leftIntensity, double rightIntensity) {
        if (Robot.isReal()) {
            controller.setRumble(PS4Controller.RumbleType.kLeftRumble, leftIntensity);
            controller.setRumble(PS4Controller.RumbleType.kRightRumble, rightIntensity);
        }
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
            driveSubsystem.getSwerveDrive().spinnyBoi(Math.PI / 2);
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
            setRumble(0, 0);
            return;
        }

        //rotation += getRightX() * 0.1;

        Translation2d driveTranslation = new Translation2d(
            getForward(),
            getStrafe()
        );

        setRumble(getStrafe() < 0.1 ? Math.abs(getStrafe()) / 2 : 0, getStrafe() > 0.1 ? Math.abs(getStrafe()) / 2 : 0);

        driveTranslation = driveTranslation.times(maxSpeed);

        driveSubsystem.getSwerveDrive().drive(driveTranslation, getRightY(), false, true);
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
