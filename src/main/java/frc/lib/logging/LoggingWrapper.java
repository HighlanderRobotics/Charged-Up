package frc.lib.logging;
import java.util.ArrayList;
import java.util.Date;
import java.util.Hashtable;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DataLogManager;

public class LoggingWrapper {

    public static final LoggingWrapper shared = new LoggingWrapper();

    // Starts the DataLogManager
    private LoggingWrapper() {
        DataLogManager.start();

        while (true) {
            // Publishes the queue
            queue.forEach((key, queuedElement) -> {
                try {
                    queuedElement.publish();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            });

            // Removes the queue
            queue.clear();
        }
    }

    private Hashtable<String, QueuedElement> queue = new Hashtable<String, QueuedElement>();

    private long queuedItemTimeToLive = 100;

    /// Logs the number immediately and Wraps into a LoggingQueue
    /// then adds it to the queue to be sent over the Network Tables
    public void add(String key, double value) {
        addToQueue(key, value);
    }

    /// Logs the data immediately and Wraps into a LoggingQueue
    /// then adds it to the queue to be sent over the Network Tables
    public void add(String key, Sendable data) {
        addToQueue(key, data);
    }

    /// Logs the boolean immediately and Wraps into a LoggingQueue
    /// then adds it to the queue to be sent over the Network Tables
    public void add(String key, boolean value) {
        addToQueue(key, value);
    }

    /// Logs the value immediately and Wraps into a LoggingQueue
    /// thenY adds it to the queue to be sent over the Network Tables
    public void add(String key, String value) {
        addToQueue(key, value);
    }

    /// Adds the Queued Element to the queue. Value's type is erased.

    private void addToQueue(String key, Object value) {
        // var existingQueuedItems = queue.stream()
        //     .filter((queuedItem) -> queuedItem.key.equals(key))
        //     .collect(Collectors.toList());

        // if (existingQueuedItems.isEmpty()) {
        //     addToQueue(key, value);
        // } else if (new Date().getTime() - existingQueuedItems.get(0).timestamp > queuedItemTimeToLive) {
        //     queue.removeAll(existingQueuedItems);
        //     addToQueue(key, value);
        // } else {
        //     System.out.println("Not logging " + key + " because the previous value is still in the queue.");
        // }
        queue.put(key, new QueuedElement(key, value));
    }
}