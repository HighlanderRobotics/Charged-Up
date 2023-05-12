package frc.lib.logging;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;
import java.util.*;

public class LoggingWrapper {

  public static final LoggingWrapper shared = new LoggingWrapper();

  private LoggingWrapper() {}

  private Hashtable<String, QueuedElement> queue = new Hashtable<String, QueuedElement>();

  /** Starts the WPILib DataLogManager to the default directory location */
  public void startLogging() {
    DataLogManager.start();
  }

  public void resetQueue() {
    queue = new Hashtable<String, QueuedElement>();
  }

  public void publishEntireQueue() {
    queue.forEach(
        (key, queuedElement) -> {
          try {
            queuedElement.publish();
          } catch (Exception e) {
            e.printStackTrace();
          }
        });
  }

  /**
   * Logs the number immediately and Wraps into a LoggingQueue then adds it to the queue to be sent
   * over the Network Tables
   */
  public void add(String key, double value) {
    addToQueue(key, value);
  }

  /**
   * Logs the data immediately and Wraps into a LoggingQueue then adds it to the queue to be sent
   * over the Network Tables
   */
  public void add(String key, Sendable data) {
    addToQueue(key, data);
  }

  /**
   * Logs the data immediately and Wraps into a LoggingQueue then adds it to the queue to be sent
   * over the Network Tables
   */
  public void add(String key, boolean value) {
    addToQueue(key, value);
  }

  /**
   * Logs the data immediately and Wraps into a LoggingQueue then adds it to the queue to be sent
   * over the Network Tables
   */
  public void add(String key, String value) {
    addToQueue(key, value);
  }

  /** Adds the Queued Element to the queue. Value's type is not taken into consideration. */
  private void addToQueue(String key, Object value) {
    queue.put(key, new QueuedElement(key, value));
  }
}
