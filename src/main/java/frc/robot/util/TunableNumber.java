package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class TunableNumber {
    private final String name;
    private final double defaultValue;
    private final DoubleSubscriber subscriber;
    private final DoublePublisher publisher;

    /**
     * Creates a tunable number that will appear under the given NT table path.
     *
     * @param tablePath     NetworkTables path (e.g. "Debug", "Shooter", etc.)
     * @param name          Name of the value in the table
     * @param defaultValue  Default value used if NT has none set yet
     */
    public TunableNumber(String tablePath, String name, double defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;

        NetworkTable table = NetworkTableInstance.getDefault().getTable(tablePath);

        publisher = table.getDoubleTopic(name).publish();
        publisher.set(defaultValue);  // initialize on dashboard

        subscriber = table.getDoubleTopic(name).subscribe(defaultValue);
    }

    /** Returns the live value from the dashboard (or the default if none). */
    public double get() {
        return subscriber.get(defaultValue);
    }

    /** Pushes a new value to the dashboard. */
    public void set(double value) {
        publisher.set(value);
    }

    /** Gets the default used for initialization. */
    public double getDefault() {
        return defaultValue;
    }

    @Override
    public String toString() {
        return name + ": " + get();
    }
}
