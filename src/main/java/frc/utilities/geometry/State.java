package frc.utilities.geometry;

import frc.utilities.util.CSVWritable;
import frc.utilities.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
