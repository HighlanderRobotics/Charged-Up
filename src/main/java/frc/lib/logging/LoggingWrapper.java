package frc.lib.logging;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggingWrapper {

    public static final LoggingWrapper shared = new LoggingWrapper();

    // Starts the DataLogManager
    private LoggingWrapper() {
        DataLogManager.start();

        // monitor CPU Usage
        
        
    }

    private List<LogElement> queue = new ArrayList<LogElement>();

    /// Logs the number immediately and Wraps into a LoggingQueue
    /// and adds it to the queue to be sent over the Network Tables
    public void add(String key, double value) {

        addToQueue(key, value);
    }

    /// Logs the data immediately and Wraps into a LoggingQueue
    /// and adds it to the queue to be sent over the Network Tables
    public void add(String key, Sendable data) {
        addToQueue(key, data);
    }

    /// Logs the boolean immediately and Wraps into a LoggingQueue
    /// and adds it to the queue to be sent over the Network Tables
    public void add(String key, boolean value) {
        addToQueue(key, value);
    }

    /// Logs the value immediately and Wraps into a LoggingQueue
    /// and adds it to the queue to be sent over the Network Tables
    public void add(String key, String value) {
        addToQueue(key, value);
    }

    private void addToQueue(String key, Object value) {
        addToQueue(key, value);
    }

    public void publishPendingLogs() {

    }
}