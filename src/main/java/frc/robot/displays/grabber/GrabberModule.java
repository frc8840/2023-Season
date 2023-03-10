package frc.robot.displays.grabber;

import frc.team_8840_lib.input.communication.dashboard.DashboardModule;

public class GrabberModule extends DashboardModule {
    public GrabberModule() {
        super();
    }

    @Override
    public void build() {
        startNewComponent();

        GrabberDisplay grabberDisplay = new GrabberDisplay();
        grabberDisplay.draw();

        addComponent(
            grabberDisplay
        );
    }
}
