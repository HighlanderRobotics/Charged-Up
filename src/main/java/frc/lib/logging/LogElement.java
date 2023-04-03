package frc.lib.logging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class LogElement {
    long timestamp;
    String key;
    Object data;

    public LogElement(String key, Object data) {
        this.timestamp = System.nanoTime();
        this.key = key;
        this.data = data;
    }

    public void publish() throws Exception {
        if (data instanceof Double) {
            SmartDashboard.putNumber(key, ((Double) data));
        } else if (data instanceof String) {
            SmartDashboard.putString(key, ((String) data));
        } else if (data instanceof Sendable) {
            SmartDashboard.putData(key, ((Sendable) data));
        } else if (data instanceof Boolean) {
            SmartDashboard.putBoolean(key, ((Boolean) data));
        } else {
            throw new Exception("Unexpected Data Type");
        }
    }
}