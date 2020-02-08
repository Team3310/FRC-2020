package frc.robot.utilities.geometry;

import frc.robot.utilities.util.CSVWritable;
import frc.robot.utilities.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
