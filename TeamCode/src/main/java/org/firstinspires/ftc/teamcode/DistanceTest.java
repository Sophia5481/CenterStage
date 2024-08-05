package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.FileWriter;
import java.io.IOException;

@TeleOp(name="DistanceTest", group="Iterative OpMode")
public class DistanceTest extends LinearOpMode {
    DistanceSensor distance1, distance2;

    @Override
    public void runOpMode() {
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");


        StringBuilder text1 = new StringBuilder();
        StringBuilder text2 = new StringBuilder();
        waitForStart();
        while (opModeIsActive()) {
            double value1 = distance1.getDistance(DistanceUnit.INCH);
            double value2 = distance2.getDistance(DistanceUnit.INCH);

            text1.append(value1 + "\n");
            text2.append(value2 + "\n");

            telemetry.addData("distance1 ", value1);
            telemetry.addData("distance2 ", value2);
            telemetry.addData("difference ", value1 - value2);
            telemetry.update();

        }

        if (isStopRequested()) {
            try {
                FileWriter fWriter = new FileWriter(String.format("%s/FIRST/data/mylog.txt", Environment.getExternalStorageDirectory().getAbsolutePath()));
                //fWriter.write("distance1 " + text1);
                fWriter.append("Distance 1: " + "\n");
                fWriter.append(text1);
                //fWriter.write("distance2 " + text2);
                fWriter.append("Distance 2: " + "\n");
                fWriter.append(text2);
                fWriter.close();
            }
            catch (IOException e) {
                telemetry.addData("error ", e.getMessage());
                telemetry.update();
            }
        }
    }
}
