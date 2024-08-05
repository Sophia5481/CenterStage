package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;

@TeleOp(name = "TeleOp", group = "Iterative Opmode")
public class CenterstageTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Attachments robot = new Attachments();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    private RevBlinkinLedDriver.BlinkinPattern ledPattern = Constants.openPattern;

    private double claw1Position = Constants.clawOpen;
    private double claw2Position = Constants.clawOpen;
    private boolean claw1Toggle = true;
    private boolean claw2Toggle = true;
    private int intakeState = 2;
    private double intake1Power = 0;
    private double intake2Power = 0;
    private double clawArmPosition = Constants.clawArmDrive;
    private double turnClawPosition = Constants.turnClawDown;
    private double planePosition = Constants.planeHold;
    private double pixelPosition = Constants.pixelHold;

    private double liftPower = 0;
    private boolean useLiftPower = true;
    private boolean liftModeUpdate = false;
    private boolean liftUseEnc = true;
    private int targetLiftPosition = 0;
    private int pixelDropLiftPosition = Constants.liftHigh;
    private int currentLiftPosition = 0;

    private double hangPower1 = 0;
    private double hangPower2 = 0;
    private boolean useHangPower = true;
    private boolean hangModeUpdate = false;
    private boolean hangUseEnc = true;
    private int targetHangPosition = 0;
    private int currentHangPosition1 = 0;
    private int currentHangPosition2 = 0;

    private boolean limits = true;

    private int stage = -1;
    private int counter = 0;

    StandardTrackingWheelLocalizer localizer;
    Pose2d currentPose;
    double heading;
    double fieldCentricOffset;

    private boolean timer = false;


    @Override
    public void init() {
        robot.initialize(hardwareMap, telemetry, false);
        localizer = new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>());
        localizer.setPoseEstimate(PoseStorage.currentPose);
        fieldCentricOffset = PoseStorage.fieldCentricOffset;
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        localizer.update();
        currentPose = localizer.getPoseEstimate();
        heading = -currentPose.getHeading();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(heading + fieldCentricOffset);




        double lx = 0;
        double ly = 0;
        double speedMultiplier = Constants.moveSpeed;
        double rotationMultiplier = Constants.rotSpeed;

        // D-pad
        if (gamepad1.dpad_up) {
            ly = 1;
            lx = 0;
            speedMultiplier = 0.6;
        } else if (gamepad1.dpad_down) {
            ly = -1;
            lx = 0;
            speedMultiplier = 0.6;
        }
        if (gamepad1.dpad_left) {
            lx = -1;
            ly = 0;
            speedMultiplier = 0.6;
        } else if (gamepad1.dpad_right) {
            lx = 1;
            ly = 0;
            speedMultiplier = 0.6;
        }

        // Math
        double theta = Math.atan2(lx, ly);
        double v_theta = Math.sqrt(lx * lx + ly * ly);
        double v_rotation = gamepad1.right_stick_x;

        // Drive
        if (lx != 0 || ly != 0) {
            robot.drive(theta, speedMultiplier * v_theta, rotationMultiplier * v_rotation);
        } else {
            robot.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
        }




        /* -------------------------------------------- CHANGE -------------------------------------------- */


        if (gamepad2.right_bumper && !previousGamepad2.right_bumper) {
            claw1Toggle = !claw1Toggle;
        }


        if (claw1Toggle) {
            claw1Position = Constants.clawOpen;
            claw2Position = Constants.clawOpen;
            ledPattern = Constants.openPattern;
        } else {
            claw1Position = Constants.clawClose;
            claw2Position = Constants.clawClose;
            ledPattern = Constants.closePattern;
        }

        if (gamepad2.left_bumper) {
            claw1Position = Constants.clawOpen;
        }



        if (gamepad2.b) {
            stage = 5;
        } else if (gamepad2.a) {
            stage = 0;
        }


        if (gamepad2.right_trigger > 0.2) {
            intakeState = 1;
        } else if (gamepad2.left_trigger > 0.2) {
            intakeState = 3;
        } else if (gamepad2.right_stick_button) {
            intakeState = 2;
        }

        if (gamepad2.left_stick_button) {
            clawArmPosition = Constants.clawArmDrive;
//            if (counter > 1) {
//                counter--;
//            } else if (counter == 1) {
//                clawArmPosition = Constants.clawArmDrive;
//                counter = 0;
//            } else {
//                counter = Constants.clawArmLiftDelay;
//            }
        }

        if (intakeState == 1) {
            intake1Power = -1;
            intake2Power = 1;
        } else if (intakeState == 3) {
            intake1Power = 1;
            intake2Power = -1;
        } else {
            intake1Power = 0;
            intake2Power = 0;
        }


        if (gamepad1.right_trigger > 0.2) {
            planePosition = Constants.planeRelease;
        } else if (gamepad1.left_trigger > 0.2) {
            planePosition = Constants.planeHold;
        }


        if (gamepad2.dpad_up) {
            useLiftPower = false;
            targetLiftPosition = pixelDropLiftPosition;
            liftUseEnc = true;
        } else if (gamepad2.dpad_down) {
            useLiftPower = false;
            targetLiftPosition = Constants.liftMin;
            liftUseEnc = true;
        }

        if (gamepad2.dpad_left) {
            pixelDropLiftPosition = currentLiftPosition;
            useLiftPower = false;
            targetLiftPosition = pixelDropLiftPosition;
            liftUseEnc = true;
        }
//        if (gamepad2.dpad_left) {
//            turnClawPosition = Constants.turnClaw0;
//        } else if (gamepad2.dpad_right) {
//            turnClawPosition = Constants.turnClaw180;
//        }

        if (gamepad2.y) {
            useHangPower = false;
            targetHangPosition = Constants.hangHigh;
            hangUseEnc = true;
        } else if (gamepad2.x) {
            useHangPower = false;
            targetHangPosition = Constants.hangLow;
            hangUseEnc = true;
        }



        if (gamepad1.right_bumper && !previousGamepad1.right_bumper) {
            limits = !limits;
        }


        if (stage >= 0) {
            switch (stage) {
                case 0:
                    // Bring Claw Arm Up
                    if ((clawArmPosition + Constants.clawArmSpeed) <= Constants.clawArmHigh + 0.01) { //(clawArmPosition + Constants.clawArmSpeed) <= Constants.clawArmLow
                        clawArmPosition += Constants.clawArmSpeed;
                    } else {
                        clawArmPosition = Constants.clawArmLow;
                        stage = 1;
                    }
                    break;
                case 1:
                    // Bring Claw Down
                    if ((turnClawPosition - Constants.turnClawSpeed) >= Constants.turnClaw90) {
                        turnClawPosition -= Constants.turnClawSpeed;
                    } else {
                        turnClawPosition = Constants.turnClaw90;
                        stage = 2;
                        claw1Toggle = false;
                        claw2Toggle = false;
                    }
                    break;
                case 2:
                    // Bring Arm Down
                    if ((clawArmPosition + Constants.clawArmSpeed) <= Constants.clawArmLow) { //(clawArmPosition + Constants.clawArmSpeed) <= Constants.clawArmLow
                        clawArmPosition += Constants.clawArmSpeed;
                    } else {
                        clawArmPosition = Constants.clawArmLow;
                        stage = 3;
                    }
                    break;
                case 3:
                    // Bring Claw Down
                    if ((turnClawPosition - Constants.turnClawSpeed) >= Constants.turnClawDown) {
                        turnClawPosition -= Constants.turnClawSpeed;
                        
                        counter = Constants.clawArmDownDelay;
                    } else {
                        if (counter == 1) {
                            counter = 0;

                            turnClawPosition = Constants.turnClawDown;
                            stage = 4;
                            claw1Toggle = false;
                            claw2Toggle = false;
                        } else {
                            counter--;
                        }
                    }
                    break;
                case 4:
                    if ((clawArmPosition + Constants.clawArmSpeed) <= Constants.clawArmDown) {
                        clawArmPosition += Constants.clawArmSpeed;
                    } else {
                        clawArmPosition = Constants.clawArmDown;
                        stage = -1;
                        claw1Toggle = true;
                        claw2Toggle = true;
                        intakeState = 1;
                    }
                    break;


                case 5:
                    // Bring Arm Up
                    claw1Toggle = false;
                    claw2Toggle = false;
                    intakeState = 2;
                    if ((clawArmPosition - Constants.clawArmSpeed) >= Constants.clawArmLow) {
                        clawArmPosition -= Constants.clawArmSpeed;
                    } else {
                        clawArmPosition = Constants.clawArmLow;
                        stage = 6;
                    }
                    break;
                case 6:
                    // Bring Claw Up
                    if ((turnClawPosition + Constants.turnClawSpeed) <= Constants.turnClawUp) {
                        turnClawPosition += Constants.turnClawSpeed;
                    } else {
                        turnClawPosition = Constants.turnClawUp;
                        stage = 7;
                    }
                    break;
                case 7:
                    // Bring Arm Up
                    if ((clawArmPosition - Constants.clawArmSpeed) >= Constants.clawArmTeleDrop) {
                        clawArmPosition -= Constants.clawArmSpeed;
                    } else {
                        clawArmPosition = Constants.clawArmTeleDrop;
                        stage = -1;
                    }
                    break;
            }
        }


        if (useLiftPower && liftPower == 0) {
            useLiftPower = false;
            targetLiftPosition = currentLiftPosition;
            liftUseEnc = true;
        }

        if (gamepad1.right_stick_button) {
            localizer.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
        }


        if (clawArmPosition < Constants.clawArmHigh) {
            double clawArmOffset = Constants.clawArmHigh - clawArmPosition;
            double clawArmDegrees = clawArmOffset / Constants.clawArmValPerDegree;
            double turnClawOffset = Constants.turnClawValPerDegree * clawArmDegrees;
            turnClawPosition = Constants.turnClaw180 - turnClawOffset;
        }



        double liftJoystick = -gamepad2.left_stick_y;
        if (liftJoystick > 0.12) {
            liftUseEnc = true;
            // user trying to lift up
            if (currentLiftPosition < Constants.liftMax || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftUpRatio;
            } else {
                liftPower = 0;
            }
        } else if (liftJoystick < -0.12) {
            liftUseEnc = true;
            // user trying to lift down
            if (currentLiftPosition > Constants.liftMin || !limits) {
                useLiftPower = true;
                liftPower = liftJoystick * Constants.liftDownRatio;
                if (currentLiftPosition > Constants.liftSlow) {
                    liftPower *= Constants.liftSlowRatio;
                }

                if ((currentLiftPosition <= Constants.liftClawArmFar) && clawArmPosition <= Constants.clawArmUp) {
                    liftPower = 0;
                }
            } else {
                liftPower = 0;
            }
        } else if (useLiftPower) {
            liftPower = 0;
        }



        double hangJoystick = -gamepad2.right_stick_y;
        if (hangJoystick > 0.12) {
            hangUseEnc = true;
            // user trying to lift up
            if (currentHangPosition1 > Constants.hangMax || !limits) {
                useHangPower = true;
                hangPower1 = hangJoystick * Constants.hangUpRatio * -1;
            } else {
                hangPower1 = 0;
            }

            if (currentHangPosition2 > Constants.hangMax || !limits) {
                useHangPower = true;
                hangPower2 = hangJoystick * Constants.hangUpRatio * -1;
            } else {
                hangPower2 = 0;
            }

        } else if (hangJoystick < -0.12) {
            hangUseEnc = true;
            // user trying to lift down
            if (currentHangPosition1 < Constants.hangMin || !limits) {
                useHangPower = true;
                hangPower1 = hangJoystick * Constants.hangDownRatio * -1;
            } else {
                hangPower1 = 0;
            }

            if (currentHangPosition2 < Constants.hangMin || !limits) {
                useHangPower = true;
                hangPower2 = hangJoystick * Constants.hangDownRatio * -1;
            } else {
                hangPower2 = 0;
            }

        } else if (useHangPower) {
            hangPower1 = 0;
            hangPower2 = 0;
        }



        double clawArmJoystick = gamepad2.right_stick_x;
        if (clawArmJoystick > 0.2) {
            if ((clawArmPosition - Constants.clawArmSpeed) > Constants.clawArmFar) {
                clawArmPosition -= Constants.clawArmSpeed * clawArmJoystick;
            } else {
                clawArmPosition = Constants.clawArmUp;
            }
        } else if (clawArmJoystick < -0.2) {
            if ((clawArmPosition + Constants.clawArmSpeed) < Constants.clawArmDown) {
                clawArmPosition -= Constants.clawArmSpeed * clawArmJoystick;
            } else {
                clawArmPosition = Constants.clawArmDown;
            }
        }




        previousGamepad1.copy(gamepad1);
        previousGamepad2.copy(gamepad2);

        currentLiftPosition = robot.getLiftMotorPosition();
        currentHangPosition1 = robot.getHangMotorPosition1();
        currentHangPosition2 = robot.getHangMotorPosition2();



        /* -------------------------------------------- ACTION -------------------------------------------- */

        robot.setClaw1Servo(claw1Position);
        robot.setClaw2Servo(claw2Position);
        robot.setClawArmServo(clawArmPosition);
        robot.setTurnClawServo(turnClawPosition);
        robot.setPlaneServo(planePosition);
        robot.setPixelServo(pixelPosition);
        robot.setIntake1Servo(intake1Power);
        robot.setIntake2Servo(intake2Power);
        robot.setLEDS(ledPattern);

        if (liftModeUpdate && liftUseEnc) {
            robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftModeUpdate = false;
        }

        if (useLiftPower) {
            robot.runLiftMotor(liftPower);
        } else {
            setLiftMotor(targetLiftPosition);
        }

        if (hangModeUpdate && hangUseEnc) {
            robot.hangMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.hangMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hangModeUpdate = false;
        }

        if (useHangPower) {
            robot.runHangMotor1(hangPower1);
            robot.runHangMotor2(hangPower2);
        } else {
//            setHangMotor(targetHangPosition);
            robot.setHangMotor(1, targetHangPosition);
        }


        /* -------------------------------------------- TELEMETRY -------------------------------------------- */

        telemetry.addData("Lift Joy", liftJoystick);
        telemetry.addData("Hang Joy", hangJoystick);
        telemetry.addData("Claw Arm Joy", clawArmJoystick);
        telemetry.addLine();
        telemetry.addData("Lift Pos", currentLiftPosition);
        telemetry.addData("Hang Motor 1 Pos", currentHangPosition1);
        telemetry.addData("Hang Motor 2 Pos", currentHangPosition2);
        telemetry.addData("Claw Arm Pos", clawArmPosition);
        telemetry.addData("Plane Pos", planePosition);
        telemetry.addData("Pixel Pos", pixelPosition);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addData("Limits", limits);
        telemetry.update();


    }

    void setLiftMotor(int position) {
        robot.setLiftMotor(1, position);
        liftUseEnc = false;
        liftModeUpdate = true;
    }

    void setLiftMotorPID(int position) {
        double newPower = robot.liftPIDController.calculate(
                currentLiftPosition, position
        );
        robot.runLiftMotor(newPower);
    }

//    void setLiftMotorPID(int position) {
//        //Undefined constants
//        double newPower, powDir;
//        //Initial error
//        double error = (double) (position - currentLiftPosition) / Constants.liftMax;
//        //Initial Time
//        telemetry.addData("1", "error: " + error);
//        if (Math.abs(error) > (double) (Constants.liftTolerance / Constants.liftMax)) {
//            //Setting p action
//            newPower = error * Constants.liftkPTele;
//            powDir = Math.signum(error);
//            newPower = Math.min(Math.abs(newPower), 1);
//
//            // Going down
//            if (powDir == 1) {
//                newPower = Math.max(newPower * Constants.liftDownRatio, Constants.liftMinPow);
//                if (currentLiftPosition > Constants.liftSlow) {
//                    newPower *= Constants.liftSlowRatio;
//                }
//            }
//            // Going up
//            else {
//                newPower = Math.min(-newPower, -Constants.liftMinPow - Constants.liftkF * currentLiftPosition / Constants.liftMax);
//            }
//            telemetry.addData("Lift Motor", newPower);
//            robot.runLiftMotor(newPower);
//        } else if (position != 0 && !liftModeUpdate) {
//            robot.setLiftMotor(0.5, position);
//            liftModeUpdate = true;
//            liftUseEnc = false;
//        } else if (!liftModeUpdate){
//            robot.runLiftMotor(0);
//        }
//    }

    void setHangMotor(int position) {
        //Undefined constants
        double newPower, powDir;
        //Initial error
        double error = -(position - currentHangPosition1) / Constants.hangMax;
        //Initial Time
        telemetry.addData("1", "error: " + error);
        if (Math.abs(error) > (Constants.hangTolerance / -Constants.hangMax)) {
            //Setting p action
            newPower = error * Constants.hangkPTele;
            powDir = Math.signum(error);
            newPower = Math.min(Math.abs(newPower), 1);

            // Going down
            if (powDir == 1) {
                newPower = Math.max(newPower * Constants.hangDownRatio, Constants.hangMinPow);
                if (currentLiftPosition > Constants.hangSlow) {
                    newPower *= Constants.hangSlowRatio;
                }
            }
            // Going up
            else {
                newPower = Math.min(-newPower, -Constants.hangMinPow - Constants.hangkF * currentHangPosition1 / Constants.hangMax);
            }
            telemetry.addData("Hang Motor", newPower);
            robot.runHangMotor1(newPower);
            robot.runHangMotor2(newPower);
        } else if (position != 0 && !liftModeUpdate) {
            robot.setHangMotor(0.5, position);
            hangModeUpdate = true;
            hangUseEnc = false;
        } else if (!liftModeUpdate){
            robot.runHangMotor1(0);
            robot.runHangMotor2(0);
        }
    }

}
