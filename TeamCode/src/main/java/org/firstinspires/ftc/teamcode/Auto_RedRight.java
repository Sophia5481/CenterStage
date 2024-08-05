package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto_RedBackdrop", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class Auto_RedRight extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        int propLocation = 2;
        int parkLocation = Constants.parkLoc;
        String startPos = "redRight";
        PoseStorage.fieldCentricOffset = Math.toRadians(90);
        boolean targetFound = true;

        robot.initialize(hardwareMap, telemetry, true);
        PoseConstants.redRight poses = new PoseConstants.redRight();

        while (!isStarted()) {;
            propLocation = robot.getPropLocation();
            telemetry.addLine("Location: " + propLocation);
            telemetry.update();
        }

        int finalPropLocation = propLocation;
        telemetry.addLine("Final Location: " + propLocation);
        telemetry.update();


        robot.setClaw1Servo(Constants.clawClose);
        robot.setClaw2Servo(Constants.clawClose);
        robot.setPixelServo(Constants.pixelHold);

        sleep(Constants.sleepTime);

        robot.setPoseEstimate(PoseConstants.newRedRight.start);
        raiseSystem(Constants.liftDropAuto, Constants.clawArmAutoDrop);
        robot.followTrajectoryAsync(
                robot.trajectoryBuilder(PoseConstants.newRedRight.start)
                        .lineToSplineHeading(PoseConstants.newRedRight.backdrop[finalPropLocation - 1])
                        .build()
        );
        robot.waitForIdle();

        dropPixelNoAprilTag();

        roadrunnerSleep(300);

        robot.followTrajectorySequenceAsync(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .forward(2)
                        .lineToSplineHeading(PoseConstants.newRedRight.pixel[finalPropLocation - 1])
                        .build()
        );
        roadrunnerSleep(500);
        resetSystem();
        robot.waitForIdle();

        robot.setPixelServo(Constants.pixelDrop);
        roadrunnerSleep(200);
        robot.setPixelServo(Constants.pixelHold);

        if (Constants.parkLoc == 2) {
            robot.followTrajectorySequenceAsync(
                    robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(48, -60, Math.toRadians(0)))
                            .lineToSplineHeading(new Pose2d(60, -60, Math.toRadians(0)))
                            .build()
            );
        } else {
            robot.followTrajectorySequenceAsync(
                    robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                            .lineToSplineHeading(new Pose2d(48, -12, Math.toRadians(0)))
                            .lineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)))
                            .build()
            );
        }
        robot.waitForIdle();

//        runAutoNoStack(
//                finalPropLocation,
//                startPos,
//
//                poses.start,
//                poses.startingTangent[propLocation - 1],
//
//                poses.pixel[propLocation - 1],
//                poses.pixelAngle[propLocation - 1],
//                poses.pixelApproachingTangent[propLocation - 1],
//
//                poses.afterPixelStartingTangent[propLocation - 1],
//                poses.afterPixelEndingTangent[propLocation - 1],
//                poses.afterPixel,
//                poses.afterPixelAngle[propLocation - 1],
//
//                poses.backdrop[propLocation - 1],
//                poses.backdropTangent,
//
//                poses.park[parkLocation - 1],
//                poses.parkAngle[parkLocation - 1],
//                poses.parkStartingTangent[parkLocation - 1],
//                poses.parkEndingTangent[parkLocation - 1]
//        );

        PoseStorage.currentPose = robot.getPoseEstimate();
        robot.setTurnClawServo(Constants.turnClawDown);
        robot.setClaw1Servo(Constants.clawOpen);
        sleep(2000);

    }
}
