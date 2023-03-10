package frc.robot.displays.alignment;

import org.json.JSONObject;

import frc.team_8840_lib.input.communication.dashboard.DashboardModule;
import frc.team_8840_lib.input.communication.dashboard.components.Text;

public class PlacingModule extends DashboardModule {
    public PlacingModule() {
        super();
    }

    @Override
    public void build() {
        startNewComponent();
        
        PlacingDisplay placingDisplay = new PlacingDisplay();
        placingDisplay.draw();

        Text text = new Text("Place Display");

        JSONObject style = new JSONObject();
        style.put("color", "white");

        text.setInlineStyle(style);

        addComponent(
            placingDisplay,
            text
        );
    }
}
