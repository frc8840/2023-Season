package frc.robot.displays.arm.testing;

import org.json.JSONObject;

import frc.team_8840_lib.input.communication.dashboard.DashboardModule;
import frc.team_8840_lib.input.communication.dashboard.components.Text;

public class BACHModule extends DashboardModule {
    public BACHModule() {
        super();
    }

    @Override
    public void build() {
        startNewComponent();
        
        BACHDisplay armDisplay = new BACHDisplay();
        armDisplay.draw();

        Text text = new Text("Base Angle Change Display");

        JSONObject style = new JSONObject();
        style.put("color", "white");

        text.setInlineStyle(style);

        addComponent(
            armDisplay,
            text
        );
    }
}
