package org.firstinspires.ftc.teamcode.CommandBase.NewCode.subsystems.shooter;

import org.opencv.core.MatOfPoint;

import java.util.List;

public interface PipelineLogger {
    List<MatOfPoint> getContours();
}
