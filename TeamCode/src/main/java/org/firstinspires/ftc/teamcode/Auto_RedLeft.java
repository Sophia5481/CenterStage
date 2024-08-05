package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Auto_RedAudience", group="Linear Opmode", preselectTeleOp = "TeleOp")
public class Auto_RedLeft extends AutonomousMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        int propLocation = 2;
        int parkLocation = Constants.parkLoc;
        String startPos = "redLeft";
        PoseStorage.fieldCentricOffset = Math.toRadians(90);
        boolean targetFound = true;

        robot.initialize(hardwareMap, telemetry, true);
        PoseConstants.redLeft poses = new PoseConstants.redLeft();

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

        if (Constants.cycle) {
            runAuto(
                    finalPropLocation,
                    startPos,

                    poses.start,
                    poses.startingTangent[propLocation - 1],

                    poses.pixel[propLocation - 1],
                    poses.pixelAngle[propLocation - 1],
                    poses.pixelApproachingTangent[propLocation - 1],

                    poses.afterPixel[propLocation - 1],
                    poses.afterPixelAngle[propLocation - 1],

                    poses.stack[propLocation - 1],
                    poses.stack2[propLocation - 1],
                    poses.beforeStack,
                    poses.stackAngle,
                    poses.stackApproachingTangent,
                    poses.stackLeavingTangent,

                    poses.beforeBackdrop,
                    poses.afterPixelAngle[propLocation - 1],
                    poses.backdrop[propLocation - 1],
                    poses.backdropTangent,

                    poses.park[parkLocation - 1],
                    poses.parkAngle[parkLocation - 1],
                    poses.parkStartingTangent[parkLocation - 1],
                    poses.parkEndingTangent[parkLocation - 1]
            );
        } else {
            runAutoNoStack(
                    finalPropLocation,
                    startPos,

                    poses.start,
                    poses.startingTangent[propLocation - 1],

                    poses.pixel[propLocation - 1],
                    poses.pixelAngle[propLocation - 1],
                    poses.pixelApproachingTangent[propLocation - 1],

                    poses.afterPixel[propLocation - 1],
                    poses.afterPixelAngle[propLocation - 1],

                    poses.stack[propLocation - 1],
                    poses.stackAngle,
                    poses.stackApproachingTangent,

                    poses.beforeBackdrop,
                    poses.afterPixelAngle[propLocation - 1],
                    poses.backdrop[propLocation - 1],
                    poses.backdropTangent,

                    poses.park[parkLocation - 1],
                    poses.parkAngle[parkLocation - 1],
                    poses.parkStartingTangent[parkLocation - 1],
                    poses.parkEndingTangent[parkLocation - 1]
            );
        }

        PoseStorage.currentPose = robot.getPoseEstimate();
        robot.setClawArmServo(Constants.clawArmDown);
        robot.setClaw1Servo(Constants.clawOpen);
        sleep(2000);

    }
}
