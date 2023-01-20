package frc.robot.testing;

import frc.team_8840_lib.utils.logging.LogWriter;

public class EmptyLogger extends LogWriter {
    public EmptyLogger() {}

    @Override
    public void initialize() {
    }

    @Override
    public void saveLine(String line) {
    }

    @Override
    public void saveInfo(String encodedInfo) {
    }

    @Override
    public void close() {
    }
}
