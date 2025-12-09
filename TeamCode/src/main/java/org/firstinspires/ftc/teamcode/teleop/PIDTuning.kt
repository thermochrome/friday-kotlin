package org.firstinspires.ftc.teamcode.teleop

import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.gamepad.ButtonReader
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

@TeleOp(name = "AprilTag Turn PID (Degrees)")
class PIDTuningAprilTag : OpMode() {

    // --- Drive motors ---
    private lateinit var lf: MotorEx
    private lateinit var rf: MotorEx
    private lateinit var lb: MotorEx
    private lateinit var rb: MotorEx

    // --- Odometry & Limelight ---
    private lateinit var odo: GoBildaPinpointDriver
    private lateinit var limelight: Limelight3A

    // --- Gamepad ---
    private lateinit var gp: GamepadEx

    // --- PID ---
    private val pid = PIDController(
        0.05,  // P
        0.0,   // I
        0.004  // D
    )

    // --- Buttons for live tuning ---
    private lateinit var pUp: ButtonReader
    private lateinit var pDown: ButtonReader
    private lateinit var iUp: ButtonReader
    private lateinit var iDown: ButtonReader
    private lateinit var dUp: ButtonReader
    private lateinit var dDown: ButtonReader

    override fun init() {
        // --- Motors ---
        lf = MotorEx(hardwareMap, "left_front")
        rf = MotorEx(hardwareMap, "right_front")
        lb = MotorEx(hardwareMap, "left_back")
        rb = MotorEx(hardwareMap, "right_back")

        // --- Odometry & Limelight ---
        odo = hardwareMap.get(GoBildaPinpointDriver::class.java, "odometry")
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0) // make sure pipeline is active

        // --- Gamepad ---
        gp = GamepadEx(gamepad1)

        // --- PID tuning buttons ---
        pUp = ButtonReader(gp, GamepadKeys.Button.DPAD_UP)
        pDown = ButtonReader(gp, GamepadKeys.Button.DPAD_DOWN)
        iUp = ButtonReader(gp, GamepadKeys.Button.RIGHT_BUMPER)
        iDown = ButtonReader(gp, GamepadKeys.Button.LEFT_BUMPER)
        dUp = ButtonReader(gp, GamepadKeys.Button.DPAD_RIGHT)
        dDown = ButtonReader(gp, GamepadKeys.Button.DPAD_LEFT)

        telemetry.addLine("Turn PID Tuning with AprilTag Ready")
    }

    override fun loop() {
        // --- Read buttons ---
        pUp.readValue(); pDown.readValue()
        iUp.readValue(); iDown.readValue()
        dUp.readValue(); dDown.readValue()

        // --- Adjust PID live ---
        if (pUp.wasJustPressed()) pid.p += 0.01
        if (pDown.wasJustPressed()) pid.p -= 0.01
        if (iUp.wasJustPressed()) pid.i += 0.0005
        if (iDown.wasJustPressed()) pid.i -= 0.0005
        if (dUp.wasJustPressed()) pid.d += 0.001
        if (dDown.wasJustPressed()) pid.d -= 0.001

        // --- Read odometry heading ---
        odo.update()
        val currentDeg = normalizeDegrees(odo.getHeadingDegrees())

        // --- Limelight AprilTag Yaw ---
        val tagYawDeg = limelight.latestResult.botpose.orientation.yaw // Limelight yaw in degrees relative to robot
        val targetDeg = currentDeg + tagYawDeg // convert to absolute field heading

        // --- PID computation ---
        val errorDeg = angleErrorDegrees(targetDeg, currentDeg)
        val output = pid.calculate(0.0, errorDeg)
        val turnPower = output.coerceIn(-0.25, 0.25)

        // --- Rotate robot ---
        lf.set(turnPower)
        lb.set(turnPower)
        rf.set(-turnPower)
        rb.set(-turnPower)

        // --- Telemetry ---
        telemetry.addData("Target (deg)", targetDeg)
        telemetry.addData("Current (deg)", currentDeg)
        telemetry.addData("Error (deg)", errorDeg)
        telemetry.addLine()
        telemetry.addData("P", pid.p)
        telemetry.addData("I", pid.i)
        telemetry.addData("D", pid.d)
        telemetry.addData("Turn Power", turnPower)
        telemetry.addData("Tag Yaw", tagYawDeg)
        telemetry.update()
    }

    // --- Normalize degrees to [-180, 180] ---
    private fun normalizeDegrees(angle: Double): Double =
        atan2(sin(Math.toRadians(angle)), cos(Math.toRadians(angle))) * 180.0 / Math.PI

    // --- Shortest-path error in degrees ---
    private fun angleErrorDegrees(target: Double, current: Double): Double {
        var error = target - current
        while (error > 180) error -= 360
        while (error < -180) error += 360
        return error
    }

    // --- Extension: get heading in degrees ---
    private fun GoBildaPinpointDriver.getHeadingDegrees(): Double =
        this.getHeading(AngleUnit.DEGREES)
}
