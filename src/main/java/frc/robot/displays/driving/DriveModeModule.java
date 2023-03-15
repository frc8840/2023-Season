package frc.robot.displays.driving;

import org.json.JSONObject;

import frc.team_8840_lib.input.communication.dashboard.DashboardModule;
import frc.team_8840_lib.input.communication.dashboard.components.Text;

public class DriveModeModule extends DashboardModule {
    public DriveModeModule() {
        super();
    }

    @Override
    public void build() {
        startNewComponent();
        
        DriverModeDisplay driveModeDisplay = new DriverModeDisplay();
        driveModeDisplay.draw();

        Text text = new Text("Drive Mode");
        Text text2 = new Text("If not white,");
        Text text3 = new Text("press same");
        Text text4 = new Text("color button");
        Text text5 = new Text("on XBOX");

        JSONObject style = new JSONObject();
        style.put("color", "white");

        text.setInlineStyle(style);
        text2.setInlineStyle(style);
        text3.setInlineStyle(style);
        text4.setInlineStyle(style);
        text5.setInlineStyle(style);

        addComponent(
            driveModeDisplay,
            text,
            text2,
            text3,
            text4,
            text5
        );
    }
}
