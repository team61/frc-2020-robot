package lib.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;

public interface Pose2dConsumer {

    void accept(Pose2d pose2d);
}
