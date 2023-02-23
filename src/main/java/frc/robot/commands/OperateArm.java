package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.displays.arm.ArmDisplay;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.utils.ArmSettings;
import frc.robot.utils.Measurements;
import frc.team_8840_lib.input.communication.CommunicationManager;

public class OperateArm extends CommandBase {
    private ArmSubsystem armSubsystem;
    private boolean lastMouseStatus = false;
    private boolean flippedArm = false;

    public OperateArm(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        armSubsystem.reportToNetworkTables();

        NetworkTableEntry mouseX = CommunicationManager.getInstance().get("custom_component", "canvas/mouse_x");
        NetworkTableEntry mouseY = CommunicationManager.getInstance().get("custom_component", "canvas/mouse_y");

        double rawX = mouseX.getDouble(0);
        double rawY = mouseY.getDouble(0) + (Units.metersToInches(Measurements.Arm.HEIGHT_FROM_GROUND) * 3.5) + 10;

        double x = Units.inchesToMeters(((ArmDisplay.WIDTH / 2) - rawX) / 3.5);
        double y = Units.inchesToMeters((ArmDisplay.HEIGHT - rawY) / 3.5);

        if (y < (-Measurements.Arm.HEIGHT_FROM_GROUND + ArmSettings.MIN_DIST_OFF_GROUND)) {
            y = (-Measurements.Arm.HEIGHT_FROM_GROUND + ArmSettings.MIN_DIST_OFF_GROUND);
        }

        if (lastMouseStatus != CommunicationManager.getInstance().get("custom_component", "canvas/mouse_down").getBoolean(false)) {
            lastMouseStatus = !lastMouseStatus;
            if (lastMouseStatus) {
                flippedArm = !flippedArm;
            }
        }

        Translation2d target = new Translation2d(-x, -y);

        Rotation2d[] angles = armSubsystem.calculatePositions(target, flippedArm);

        angles = armSubsystem.translateToRelativeAngles(angles);

        //angles = armSubsystem.magicWitchcraft(angles, target);

        armSubsystem.setPosition(angles[0], angles[1]);
    }
}
