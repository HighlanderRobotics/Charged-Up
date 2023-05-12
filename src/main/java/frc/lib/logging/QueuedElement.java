package frc.lib.logging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Date;

class QueuedElement {

  /** Timestamp in Milliseconds */
  long timestamp;

  /** The Key of the Queued Element. This should be unique and identifiable */
  String key;

  /**
   * The object that the key is associated with. This should be a Double, String, Sendable, or
   * Boolean
   */
  Object data;

  /**
   * Constructor for a QueuedElement
   *
   * @param key The Key of the Queued Element. This should be unique and identifiable
   * @param data The object that the key is associated with. This should be a Double, String,
   *     Sendable, or Boolean
   */
  public QueuedElement(String key, Object data) {
    this.timestamp = new Date().getTime();
    this.key = key;
    this.data = data;
  }

  /** Publishes the Queued Element to the SmartDashboard */
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
