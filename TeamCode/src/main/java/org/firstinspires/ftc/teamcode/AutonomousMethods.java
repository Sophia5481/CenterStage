package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public abstract class AutonomousMethods extends LinearOpMode {


    public Attachments robot = new Attachments();

    FtcDashboard dashboard = FtcDashboard.getInstance();
    private Orientation angles;
    public ElapsedTime runtime = new ElapsedTime();

    TrajectoryVelocityConstraint velocityConstraint = new TranslationalVelocityConstraint(Constants.slowerSplineVel);

    public boolean opModeStatus() {
        return opModeIsActive();
    }

    public void initializeRobot(HardwareMap hardwareMap, Telemetry telemetry) {

        robot.initialize(hardwareMap, telemetry, true);
//        robot.setPixelServo(Constants.pixelHold);

    }

    public void runAutoNoStack(
            int propLocation,
            String startPos,

            Pose2d startPose,
            double startTangent,

            Vector2d pixel,
            double pixelAngle,
            double pixelApproachingTangent,

            Vector2d afterPixel,
            double afterPixelAngle,

            Vector2d stack,
            double stackAngle,
            double stackApproachingTangent,

            Vector2d beforeBackdrop,
            double beforeBackdropAngle,
            Vector2d backdrop,
            double backdropTangent,

            Vector2d park,
            double parkAngle,
            double parkStartingTangent,
            double parkEndingTangent
    ) {

        double monkey = Constants.monkey;

        robot.setPoseEstimate(startPose);
        robot.setClaw1Servo(Constants.clawOpen);
        robot.setClawArmServo(Constants.clawArmDown);
        robot.setIntakeState(Attachments.intakeState.IN);
        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(startPose)
                        .setTangent(startTangent)
                        .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                        .addDisplacementMarker(() -> {
                            robot.setPixelServo(Constants.pixelDrop);
                            roadrunnerSleep(200);
                            robot.setPixelServo(Constants.pixelHold);
                        })
                        .setVelConstraint(velocityConstraint)
                        .setTangent(stackApproachingTangent)
                        .splineToSplineHeading(new Pose2d(stack, stackAngle), stackApproachingTangent)
                        .resetVelConstraint()
                        .waitSeconds(0.5)
                        .addDisplacementMarker(() -> {
                            robot.setClaw1Servo(Constants.clawClose);
                            sleep(200);
                            robot.setIntakeState(Attachments.intakeState.OUT);
                        })
                        .back(monkey)
                        .lineTo(afterPixel)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(beforeBackdrop, beforeBackdropAngle), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.setIntakeState(Attachments.intakeState.STOP);
                            raiseSystem(Constants.liftDropAuto, Constants.clawArmAutoDrop);
                        })
                        .splineToLinearHeading(new Pose2d(backdrop, afterPixelAngle), backdropTangent)
                        .build()
        );

        roadrunnerSleep(200);

        dropPixel(true, propLocation, startPos);

        roadrunnerSleep(500);

        robot.followTrajectorySequenceAsync(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(parkStartingTangent)
                        .splineToSplineHeading(new Pose2d(park, parkAngle), parkEndingTangent)
                        .build()
        );

        roadrunnerSleep(500);
        resetSystem();
        robot.waitForIdle();

    }

    public void runAuto(
            int propLocation,
            String startPos,

            Pose2d startPose,
            double startTangent,

            Vector2d pixel,
            double pixelAngle,
            double pixelApproachingTangent,

            Vector2d afterPixel,
            double afterPixelAngle,

            Vector2d stack,
            Vector2d stack2,
            Vector2d beforeStack,
            double stackAngle,
            double stackApproachingTangent,
            double stackLeavingTangent,

            Vector2d beforeBackdrop,
            double beforeBackdropAngle,
            Vector2d backdrop,
            double backdropTangent,
            Vector2d park,
            double parkAngle,
            double parkStartingTangent,
            double parkEndingTangent
    ) {

        robot.setPoseEstimate(startPose);
        robot.setClaw1Servo(Constants.clawOpen);
        robot.setClawArmServo(Constants.clawArmDown);
        robot.setIntakeState(Attachments.intakeState.IN);
        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(startPose)
                        .setTangent(startTangent)
                        .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                        .addDisplacementMarker(() -> {
                            robot.setPixelServo(Constants.pixelDrop);
                            roadrunnerSleep(200);
                            robot.setPixelServo(Constants.pixelHold);
                        })

                        .setVelConstraint(velocityConstraint)
                        .setTangent(stackApproachingTangent)
                        .splineToSplineHeading(new Pose2d(stack, stackAngle), stackApproachingTangent)
                        .resetVelConstraint()
                        .waitSeconds(0.5)
                        .addDisplacementMarker(() -> {
                            robot.setClaw1Servo(Constants.clawClose);
                            sleep(200);
                            robot.setIntakeState(Attachments.intakeState.OUT);
                        })
                        .back(0.5)
                        .lineTo(afterPixel)
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(beforeBackdrop, beforeBackdropAngle), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.setIntakeState(Attachments.intakeState.STOP);
                            raiseSystem(Constants.liftDropAuto, Constants.clawArmAutoDrop);
                        })
                        .splineToLinearHeading(new Pose2d(backdrop, afterPixelAngle), backdropTangent)
                        .build()
        );

        roadrunnerSleep(200);

        dropPixel(true, propLocation, startPos);

        roadrunnerSleep(100);

        int tan = -90;
        if (startPos.startsWith("red")) {
            tan = 90;
        }

        robot.followTrajectorySequenceAsync(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(Math.toRadians(tan))
                        .splineToSplineHeading(new Pose2d(beforeBackdrop, afterPixelAngle), stackApproachingTangent)
                        .splineToSplineHeading(new Pose2d(beforeStack, afterPixelAngle), stackApproachingTangent)
                        .setVelConstraint(velocityConstraint)
                        .splineToSplineHeading(new Pose2d(stack2, stackAngle), stackApproachingTangent)
                        .build()
        );
        resetSystem();
        robot.setIntakeState(Attachments.intakeState.IN);
        robot.waitForIdle();

        roadrunnerSleep(500);
//        robot.followTrajectorySequence(
//                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
//                        .back(1)
//                        .forward(1)
//                        .build()
//        );
        roadrunnerSleep(500);

        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(stackLeavingTangent)
                        .back(4)
                        .waitSeconds(0.2)
                        .addDisplacementMarker(() -> {
                            robot.setClaw1Servo(Constants.clawClose);
                            robot.setClaw2Servo(Constants.clawClose);
                            robot.setIntakeState(Attachments.intakeState.OUT);
                        })
                        .splineToSplineHeading(new Pose2d(beforeBackdrop, afterPixelAngle), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.setIntakeState(Attachments.intakeState.STOP);
                            raiseSystem(Constants.liftDropAuto + 300, Constants.clawArmAutoDrop);
                        })
                        .splineToSplineHeading(new Pose2d(backdrop, afterPixelAngle), backdropTangent)
                        .build()
        );

        dropPixel(false, propLocation, startPos);

        robot.followTrajectorySequenceAsync(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(parkStartingTangent)
                        .splineToSplineHeading(new Pose2d(park, parkAngle), parkEndingTangent)
                        .build()
        );

        roadrunnerSleep(500);
        resetSystem();
        robot.waitForIdle();

    }

    public void runAutoNoStack(
            int propLocation,
            String startPos,

            Pose2d startPose,
            double startTangent,

            Vector2d pixel,
            double pixelAngle,
            double pixelApproachingTangent,

            double afterPixelStartingTangent,
            double afterPixelEndingTangent,
            Vector2d afterPixel,
            double afterPixelAngle,

            Vector2d backdrop,
            double backdropTangent,

            Vector2d park,
            double parkAngle,
            double parkStartingTangent,
            double parkEndingTangent
    ) {

        robot.setPoseEstimate(startPose);
        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(startPose)
                        .setTangent(startTangent)
                        .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                        .addDisplacementMarker(() -> {
                            robot.setPixelServo(Constants.pixelDrop);
                            roadrunnerSleep(200);
                            robot.setPixelServo(Constants.pixelHold);
                        })
                        .setTangent(afterPixelStartingTangent)
                        .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
//                        .waitSeconds(10)
                        .addDisplacementMarker(() -> {
                            raiseSystem(Constants.liftDropAuto, Constants.clawArmAutoDrop);
                        })
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                        .build()
        );

        dropPixel(true, propLocation, startPos);

        robot.followTrajectorySequenceAsync(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(parkStartingTangent)
                        .splineToSplineHeading(new Pose2d(park, parkAngle), parkEndingTangent)
                        .build()
        );

        roadrunnerSleep(500);
        resetSystem();
        robot.waitForIdle();

    }

    public void runAuto(
            int propLocation,
            String startPos,

            Pose2d startPose,
            double startTangent,

            Vector2d pixel,
            double pixelAngle,
            double pixelApproachingTangent,

            double afterPixelStartingTangent,
            double afterPixelEndingTangent,
            Vector2d afterPixel,
            double afterPixelAngle,

            Vector2d stack2,
            Vector2d beforeStack,
            double stackAngle,
            double stackApproachingTangent,
            double stackLeavingTangent,

            Vector2d beforeBackdrop,
            Vector2d backdrop,
            double backdropTangent,

            Vector2d park,
            double parkAngle,
            double parkStartingTangent,
            double parkEndingTangent
    ) {

        robot.setPoseEstimate(startPose);
        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(startPose)
                        .setTangent(startTangent)
                        .splineToSplineHeading(new Pose2d(pixel, pixelAngle), pixelApproachingTangent)
                        .addDisplacementMarker(() -> {
                            robot.setPixelServo(Constants.pixelDrop);
                            roadrunnerSleep(200);
                            robot.setPixelServo(Constants.pixelHold);
                        })
                        .setTangent(afterPixelStartingTangent)
                        .splineToSplineHeading(new Pose2d(afterPixel, afterPixelAngle), afterPixelEndingTangent)
                        .addDisplacementMarker(() -> {
                            raiseSystem(Constants.liftDropAuto, Constants.clawArmAutoDrop);
                        })
                        .setTangent(Math.toRadians(0))
                        .splineToSplineHeading(new Pose2d(backdrop, Math.toRadians(180)), backdropTangent)
                        .build()
        );

        dropPixel(true, propLocation, startPos);

        robot.followTrajectorySequenceAsync(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(Math.toRadians(-100))
                        .splineToSplineHeading(new Pose2d(beforeBackdrop, afterPixelAngle), stackApproachingTangent)
                        .splineToSplineHeading(new Pose2d(beforeStack, afterPixelAngle), stackApproachingTangent)
                        .setVelConstraint(velocityConstraint)
                        .splineToSplineHeading(new Pose2d(stack2, stackAngle), stackApproachingTangent)
                        .build()
        );

        roadrunnerSleep(500);
        resetSystem();
        robot.setIntakeState(Attachments.intakeState.IN);
        robot.waitForIdle();

        roadrunnerSleep(1000);

        robot.followTrajectorySequence(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(stackLeavingTangent)
                        .back(4)
                        .waitSeconds(0.2)
                        .addDisplacementMarker(() -> {
                            robot.setClaw1Servo(Constants.clawClose);
                            robot.setClaw2Servo(Constants.clawClose);
                            robot.setIntakeState(Attachments.intakeState.OUT);
                        })
                        .splineToSplineHeading(new Pose2d(beforeBackdrop, afterPixelAngle), Math.toRadians(0))
                        .addDisplacementMarker(() -> {
                            robot.setIntakeState(Attachments.intakeState.STOP);
                            raiseSystem(Constants.liftDropAuto + 200, Constants.clawArmAutoDrop);
                        })
                        .splineToSplineHeading(new Pose2d(backdrop, stackAngle), backdropTangent)
                        .build()
        );

        dropPixel(false, propLocation, startPos);

        robot.followTrajectorySequenceAsync(
                robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                        .setTangent(parkStartingTangent)
                        .splineToSplineHeading(new Pose2d(park, parkAngle), parkEndingTangent)
                        .build()
        );

        roadrunnerSleep(500);
        resetSystem();
        robot.waitForIdle();

    }

    public void dropPixel(boolean aprilTag, int propLocation, String startPos) {
        boolean targetFound;

        if (aprilTag) {
            targetFound = goToAprilTag(propLocation, startPos, robot.visionPortal, robot.aprilTagProcessor);
        } else {
            targetFound = goToBackdrop(robot.visionPortal, robot.aprilTagProcessor);
        }

        if (targetFound) {
            if (aprilTag) {
                robot.setClaw2Servo(Constants.clawOpen);
                robot.followTrajectorySequence(
                        robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                .forward(3)
                                .addDisplacementMarker(() -> {
                                    robot.setLiftMotor(1, Constants.liftDropAuto + 350);
                                })
                                .back(3)
                                .build()
                );
                if (propLocation == 1 || propLocation == 2) {
                    robot.followTrajectory(
                            robot.trajectoryBuilder(robot.getPoseEstimate())
                                    .strafeLeft(4)
                                    .build()
                    );
                } else {
                    robot.followTrajectory(
                            robot.trajectoryBuilder(robot.getPoseEstimate())
                                    .strafeRight(4)
                                    .build()
                    );
                }
            }
            robot.setClaw2Servo(Constants.clawOpen);
            robot.setClaw1Servo(Constants.clawOpen);
        }
    }

    public void dropPixelNoAprilTag() {
            robot.setClaw2Servo(Constants.clawOpen);
            robot.setClaw1Servo(Constants.clawOpen);
    }

    public void resetSystem() {
        robot.setLiftMotor(1, Constants.liftMin);
        robot.setClawArmServo(Constants.clawArmLow);
        roadrunnerSleep(1000);
        robot.setTurnClawServo(Constants.turnClawDown);
        roadrunnerSleep(500);
        robot.setClawArmServo(Constants.clawArmDown);
    }

    public void raiseSystem(int liftPos, double clawArmPos) {
        robot.setClawArmServo(clawArmPos);
        roadrunnerSleep(300);

        double clawArmOffset = Constants.clawArmHigh - Constants.clawArmAutoDrop;
        double clawArmDegrees = clawArmOffset / Constants.clawArmValPerDegree;
        double turnClawOffset = Constants.turnClawValPerDegree * clawArmDegrees;
        double turnClawPosition = Constants.turnClaw180 - turnClawOffset;

        telemetry.addLine(String.valueOf(turnClawPosition));
        telemetry.update();
        robot.setTurnClawServo(turnClawPosition);
        robot.setLiftMotor(0.5, liftPos);
    }

    /*
     * Sleeps for some amount of milliseconds while updating the roadrunner position
     */
    public void roadrunnerSleep(int milliseconds) {
        long timeStamp = runtime.now(TimeUnit.MILLISECONDS);
        while (runtime.now(TimeUnit.MILLISECONDS) - timeStamp <= milliseconds) {
            robot.update();
        }
    }

    public void relocalize(int propLocation, double x, double y) {
        Vector2d newVector = null;
        if (propLocation == 1) {
            newVector = new Vector2d(PoseConstants.apriltags.one.getX() - x, PoseConstants.apriltags.one.getY() + y);
        } else if (propLocation == 2) {
            newVector = new Vector2d(PoseConstants.apriltags.two.getX() - x, PoseConstants.apriltags.two.getY() + y);
        } else if (propLocation == 3) {
            newVector = new Vector2d(PoseConstants.apriltags.three.getX() - x, PoseConstants.apriltags.three.getY() + y);
        } else if (propLocation == 4) {
            newVector = new Vector2d(PoseConstants.apriltags.four.getX() - x, PoseConstants.apriltags.four.getY() + y);
        } else if (propLocation == 5) {
            newVector = new Vector2d(PoseConstants.apriltags.five.getX() - x, PoseConstants.apriltags.five.getY() + y);
        } else if (propLocation == 6) {
            newVector = new Vector2d(PoseConstants.apriltags.six.getX() - x, PoseConstants.apriltags.six.getY() + y);
        }

        assert newVector != null;
        robot.setPoseEstimate(new Pose2d(newVector, robot.getPoseEstimate().getHeading()));
    }

    public boolean goToAprilTag(int propLocation, String startPosition, VisionPortal visionPortal, AprilTagProcessor aprilTagProcessor) {
        telemetry.addData("Made it: ", "to goToAprilTag");
        telemetry.update();
        final double DESIRED_DISTANCE = Constants.DESIRED_DISTANCE;

        final boolean USE_WEBCAM = true;
        int DESIRED_TAG_ID = propLocation;
        if (startPosition.equals("redLeft") || startPosition.equals("redRight")) {
            DESIRED_TAG_ID += 3;
        }
        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        /* -------------------------------------------- APRIL TAG DETECTION -------------------------------------------- */

        if (USE_WEBCAM) {
            setManualExposure(6, 250, visionPortal);
        }

        telemetry.addData("Finished ", "initialization");
        telemetry.update();

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata != null) {
                //  Check to see if we want to track towards this tag.
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    // Yes, we want to use this tag.
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }
        }
        telemetry.addData("targetFound ", targetFound);
        telemetry.update();


        /* -------------------------------------------- MOVEMENT -------------------------------------------- */

        if (targetFound) { //should add timer
            telemetry.addLine("Detected");

            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE) * -1;
            double xError = (desiredTag.ftcPose.x - Constants.APRIL_TAG_OFFSET) * -1;

            relocalize(DESIRED_TAG_ID, desiredTag.ftcPose.range + Constants.CAMERA_TO_CENTER, desiredTag.ftcPose.x);

            telemetry.addData("rangeError ", rangeError);
            telemetry.addData("yawError", xError);
            telemetry.update();

            Pose2d currentPose = robot.getPoseEstimate();
            robot.followTrajectorySequence(
                    robot.trajectorySequenceBuilder(currentPose)
                            .forward(rangeError)
                            .strafeRight(xError)
                            .build()
            );

            return true;

        } else { //does not detect so use roadrunner
            telemetry.addLine("Not Detected");
            return false;

        }

    }

    public boolean goToBackdrop(VisionPortal visionPortal, AprilTagProcessor aprilTagProcessor) {
        telemetry.addData("Made it: ", "to goToAprilTag");
        telemetry.update();
        final double DESIRED_DISTANCE = Constants.DESIRED_DISTANCE;

        final boolean USE_WEBCAM = true;
        AprilTagDetection desiredTag = null;
        boolean targetFound = false;

        /* -------------------------------------------- APRIL TAG DETECTION -------------------------------------------- */

        if (USE_WEBCAM) {
            setManualExposure(6, 250, visionPortal);
        }

        telemetry.addData("Finished ", "initialization");
        telemetry.update();

        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections.size() != 0) {
            targetFound = true;
            desiredTag = currentDetections.get(0);
        }

        /* -------------------------------------------- MOVEMENT -------------------------------------------- */

        if (targetFound) { //should add timer
            telemetry.addLine("Detected");

            double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE) * -1;
            double xError = desiredTag.ftcPose.x * -1;

            telemetry.addData("rangeError ", rangeError);
            telemetry.addData("yawError", xError);
            telemetry.update();

            Pose2d currentPose = robot.getPoseEstimate();
            robot.followTrajectorySequence(
                    robot.trajectorySequenceBuilder(currentPose)
                            .forward(rangeError)
                            .build()
            );
            return true;

        } else { //does not detect so use roadrunner
            telemetry.addLine("Not Detected");
            return false;

        }

    }



    private Pose2d calcNewPose(double x, double y, Pose2d currentPose) {
        double newX = currentPose.getX() + x;
        double newY = currentPose.getY() - y;
        return new Pose2d(newX, newY, Math.toRadians(180));
    }

    private void setManualExposure(int exposureMS, int gain, VisionPortal visionPortal) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    public void moveRobotAprilTag(double x, double y, double yaw, DcMotor leftFrontDrive, DcMotor rightFrontDrive, DcMotor leftBackDrive, DcMotor rightBackDrive) {
        telemetry.addData("inside moveRobotAprilTag = ", "true");
//        telemetry.update();
        // Calculate wheel powers.
        // Calculate wheel powers.
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        sleep(500);
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void roadrunnerToBackdrop(int propLocation, String startPosition) {
        Vector2d toBackDrop = null;

        if (startPosition.equals("blueRight") || startPosition.equals("blueLeft")) {
            toBackDrop = (propLocation == 1) ? PoseConstants.backDropBlueRight.left : ((propLocation == 2) ? PoseConstants.backDropBlueRight.center : PoseConstants.backDropBlueRight.right);
        } else if (startPosition.equals("redRight") || startPosition.equals("redLeft")) {
            toBackDrop = (propLocation == 4) ? PoseConstants.backDropRedRight.left : ((propLocation == 5) ? PoseConstants.backDropRedRight.center : PoseConstants.backDropRedRight.right);
        }

        Pose2d currentPose = robot.getPoseEstimate();
        Trajectory backdropTraj = robot.trajectoryBuilder(currentPose)
                .lineTo(toBackDrop)
                //.splineToSplineHeading(new Pose2d(new Vector2d(46, 29), Math.toRadians(180)), Math.toRadians(180))
                .build();
        robot.followTrajectory(backdropTraj);

    }

}