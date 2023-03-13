package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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

public class XboxDrive extends CommandBase {
    public static enum DriveMode {
        NORMAL,
        X_BRAKE,
        ZEROED,
        SPINNY_BOI,
        TESTING;
        public boolean normalOr(DriveMode driveMode) {
            return this == NORMAL || this == driveMode;
        }
    }

    private final static double maxSpeed = 3;

    private XboxController controller;
    private SimulatedController simulatedController;

    private DriveSubsystem driveSubsystem;

    private DriveMode driveMode = DriveMode.NORMAL;

    private Trigger xModeTrigger; //Binded to cross button
    private Trigger zeroModeTrigger; //Binded to square button
    private Trigger spinnyBoiTrigger; //Binded to circle button
    private Trigger testTrigger;

    private double rotation = 0;

    public XboxDrive(DriveSubsystem driveSubsystem) {
        //Setup Controllers
        if (Robot.isReal()) this.controller = new XboxController(ControllerConstants.DRIVE_XBOX_CONTROLLER);
        this.simulatedController = new SimulatedController();

        //Setup Drive Subsystem
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);

        if (Robot.isReal()) {
            //Add Triggers to Controllers
            xModeTrigger = new Trigger(controller::getXButton).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.X_BRAKE)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.X_BRAKE : DriveMode.NORMAL;
                    adjustBrakeModeBasedOnMode();
                })
            );

            zeroModeTrigger = new Trigger(controller::getBButton).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.ZEROED)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.ZEROED : DriveMode.NORMAL;
                })
            );

            spinnyBoiTrigger = new Trigger(controller::getAButton).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.SPINNY_BOI)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.SPINNY_BOI : DriveMode.NORMAL;
                })
            );

            testTrigger = new Trigger(controller::getYButton).onTrue(
                Commands.runOnce(() -> {
                    if (!driveMode.normalOr(DriveMode.TESTING)) return;
                    driveMode = driveMode == DriveMode.NORMAL ? DriveMode.TESTING : DriveMode.NORMAL;
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
        return Robot.isSimulation() ? 0 : controller.getRightX();
    }

    public double getRightY() {
        return Robot.isSimulation() ? 0 : controller.getRightY();
    }

    @Override
    public void initialize() {
        Logger.Log("[" + getName() + "] Initialized.");
    }

    private int count = 0;
    private double testrot = 0;

    @Override
    public void execute() {
        if (GamePhase.getCurrentPhase() != GamePhase.Teleop) return;
        
        SmartDashboard.putString("Drive Mode", driveMode.name());

        if (driveMode == DriveMode.SPINNY_BOI) {
            driveSubsystem.getSwerveDrive().spin(controller.getRightX() * 18);
            return;
        } else if (driveMode == DriveMode.X_BRAKE) {
            driveSubsystem.getSwerveDrive().applyXBrake();
            driveSubsystem.getSwerveDrive().stop();
            return;
        } else if (driveMode == DriveMode.ZEROED) {
            driveSubsystem.getSwerveDrive().setAllModuleAngles(0);
            driveSubsystem.getSwerveDrive().stop();
            return;
        } else if (driveMode == DriveMode.TESTING) {
            if (controller.getLeftBumperPressed()) {
                count += 1;
                testrot = 0;
            }

            int index = count % 4;

            if (controller.getRightBumperPressed()) {
                testrot += 1;
            }

            driveSubsystem.getSwerveDrive().getModules()[index].setDesiredState(
                new SwerveModuleState(0.5, Rotation2d.fromDegrees(testrot))
            , true, true);
            
            controller.setRumble(RumbleType.kBothRumble, 0.75);
            return;
        }

        if (driveSubsystem.isInAutoDrive()) return;
        
        if (Math.abs(getForward()) < 0.1 && Math.abs(getStrafe()) < 0.1) {
            if (Math.abs(getRightX()) < 0.1) {
                driveSubsystem.getSwerveDrive().stop();
            } else {
                driveSubsystem.getSwerveDrive().spin(getRightX() * 13);
            }
            return;
        }

        Translation2d driveTranslation = new Translation2d(
            getForward(),
            getStrafe()
        );

        driveTranslation = driveTranslation.times(maxSpeed);

        driveSubsystem.getSwerveDrive().drive(driveTranslation, getRightX() * 13, true, Robot.isReal());
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
