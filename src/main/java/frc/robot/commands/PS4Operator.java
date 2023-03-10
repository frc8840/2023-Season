package frc.robot.commands;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.GrabberSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.info.console.Logger;
import frc.team_8840_lib.input.communication.CommunicationManager;
import frc.team_8840_lib.listeners.Robot;
import frc.team_8840_lib.utils.math.MathUtils;
import frc.team_8840_lib.utils.math.units.RectangleBounds;

public class PS4Operator extends CommandBase {
    public enum OperateState {
        AUTO_ALIGN,
        PLACE,
        PICKUP,
        NONE;
    }

    private PS4Controller controller;

    private Trigger coneTrigger;
    private Trigger cubeTrigger;

    private Trigger switchSideTrigger;
    private Trigger selectTrigger;

    private int rowHover = 0;
    private int columnHover = 0;

    private int selectedRow = 0;
    private int selectedColumn = 0;

    private boolean justMoved = false;

    private Translation2d startAutoAlignPosition = null;

    private GrabberSubsystem grabberSubsystem;

    private OperateState state = OperateState.NONE;

    private String side = "blue";

    public PS4Operator(GrabberSubsystem grabberSubsystem) {
        controller = new PS4Controller(0);//ControllerConstants.OPERATOR_PS4_CONTROLLER);

        this.grabberSubsystem = grabberSubsystem;

        coneTrigger = new Trigger(controller::getCrossButton).onTrue(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getLoadedPiece() == GrabberSubsystem.LoadedPiece.NONE) {
                    grabberSubsystem.intake(GrabberSubsystem.LoadedPiece.CONE);
                } else {
                    grabberSubsystem.outtake();
                }

                grabberSubsystem.updateDashboard();
            })
        ).onFalse(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getState() == GrabberSubsystem.GrabberState.ACTIVE) {
                    grabberSubsystem.stopPickOrPlace();
                }

                grabberSubsystem.updateDashboard();
            })
        );

        cubeTrigger = new Trigger(controller::getSquareButton).onTrue(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getLoadedPiece() == GrabberSubsystem.LoadedPiece.NONE) {
                    grabberSubsystem.intake(GrabberSubsystem.LoadedPiece.CUBE);
                } else {
                    grabberSubsystem.outtake();
                }

                grabberSubsystem.updateDashboard();
            })
        ).onFalse(
            Commands.runOnce(() -> {
                if (grabberSubsystem.getState() == GrabberSubsystem.GrabberState.ACTIVE) {
                    grabberSubsystem.stopPickOrPlace();
                }

                grabberSubsystem.updateDashboard();
            })
        );

        switchSideTrigger = new Trigger(controller::getL2Button).onTrue(
            Commands.runOnce(() -> {
                changeSide();
            })
        );

        selectTrigger = new Trigger(controller::getL1Button).onTrue(
            Commands.runOnce(() -> {
                selectedRow = rowHover;
                selectedColumn = columnHover;
                updateSelected();
            })
        );

        new Trigger(controller::getTriangleButton).onTrue(
            Commands.runOnce(() -> {
                calculateDistanceFromSelected();
            })
        );

        addRequirements(grabberSubsystem);
    }

    @Override
    public void initialize() {
        updateSelected();

        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/side", "blue" //TODO: change depending on alliance
        );
    }

    public void updateSelected() {
        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/hover", (columnHover * 3) + rowHover
        );

        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/selected", (selectedColumn * 3) + selectedRow
        );
    }

    public boolean calculateDistanceFromSelected() {
        RectangleBounds[] positions = Measurements.Grid.Rows.getBlueDimensionsList();

        int index = (selectedColumn * 3) + selectedRow;

        boolean isLeftSide = side == "blue";

        double x = isLeftSide ? positions[index].getCenterX() : Measurements.Field.WIDTH - positions[index].getCenterX();
        double y = (Measurements.Field.HEIGHT - Measurements.Grid.DEPTH) + positions[index].getCenterY();

        if (Robot.isReal()) {
            if (VisionSubsystem.getInstance().hasTarget()) {
                EstimatedRobotPose estPose = VisionSubsystem.getInstance().getEstimatedRobotPose();
                
                startAutoAlignPosition = new Translation2d(
                    estPose.estimatedPose.getX(),
                    estPose.estimatedPose.getY()
                );
            } else {
                Logger.Log("[AutoAlign] No target found! Cannot calculate distance from target!");
                return false;
            }
        } else {
            //Get closest 
            Pose2d robotPose = RobotContainer.getInstance().getDriveSubsystem().getSwerveDrive().getPose();

            //Find the closest measurement to the robot
            int closestIndex = -1;
            double closestDistance = Double.MAX_VALUE;

            for (RectangleBounds rb : positions) {
                double distance = MathUtils.distance(
                    robotPose.getX(),
                    robotPose.getY(),
                    rb.getCenterX(),
                    rb.getCenterY()
                );

                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestIndex = (int) rb.getCenterX();
                }
            }

            if (closestIndex == -1) return false;

            double closestY = positions[closestIndex].getCenterY();

            Logger.Log("Closest Y: " + closestY + " index: " + closestIndex);

            SmartDashboard.putNumber("close", closestIndex);
        }

        return true;
    }

    public void changeSide() {
        side = side == "blue" ? "red" : "blue";
        CommunicationManager.getInstance().updateInfo(
            "custom_component", "Placing Display/side", side //TODO: change depending on alliance
        );
    }

    @Override
    public void execute() {
        //etc

        if (this.state == OperateState.AUTO_ALIGN) {
            
        }

        //Auto Align Chooser
        if (!justMoved && controller.getPOV() >= 0) {
            justMoved = true;

            if (controller.getPOV() == 270) {
                rowHover += (side == "blue" ? -1 : 1);
            } else if (controller.getPOV() == 90) {
                rowHover += (side == "blue" ? 1 : -1);
            } else if (controller.getPOV() == 0) {
                columnHover -= 1;
            } else if (controller.getPOV() == 180) {
                columnHover += 1;
            }

            if (columnHover < 0) {
                columnHover = 0;
            } else if (columnHover > 8) {
                columnHover = 8;
            }

            if (rowHover < 0) {
                rowHover = 0;
            } else if (rowHover > 2) {
                rowHover = 2;
            }

            updateSelected();
        } else if (controller.getPOV() < 0) {
            justMoved = false;
        }
    
        
    }
}