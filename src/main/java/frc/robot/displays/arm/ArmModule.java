package frc.robot.displays.arm;

import org.json.JSONObject;

import frc.team_8840_lib.input.communication.dashboard.DashboardModule;
import frc.team_8840_lib.input.communication.dashboard.components.Text;

public class ArmModule extends DashboardModule {
    public ArmModule() {
        super();
    }

    @Override
    public void build() {
        startNewComponent();
        
        ArmDisplay armDisplay = new ArmDisplay();
        armDisplay.draw();

        Text text = new Text("Arm Display");

        JSONObject style = new JSONObject();
        style.put("color", "white");

        text.setInlineStyle(style);

        addComponent(
            armDisplay,
            text
        );
    }
}
