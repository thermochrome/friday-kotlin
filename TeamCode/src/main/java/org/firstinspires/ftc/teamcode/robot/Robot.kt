package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.command.Robot
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import kotlin.math.abs
import kotlin.math.atan2

class Robot(hardwareMap: HardwareMap, gamepad1: Gamepad, gamepad2: Gamepad): Robot() {
    val hardware = Hardware(hardwareMap)
    val drive = Drive(hardware)

    val gamepads = Pair(GamepadEx(gamepad1), GamepadEx(gamepad2))

    private val limelight: Limelight3A = hardware["limelight"]
    private val odometry: GoBildaPinpointDriver = hardware["odometry"]

    private val pid = PIDController(0.5, 0.1, 0.0007).apply { setTolerance(2.0) }

    init {
        register(drive)
    }

    fun turnToAngle(targetRad: Double, maxPower: Double = 0.4) {
        val timer = ElapsedTime()

        val target = normalizeRadians(targetRad)

        while (timer.milliseconds() < 2000) {

            odometry.update()
            val current = normalizeRadians(
                odometry.getHeading(AngleUnit.RADIANS)
            )

            val error = angleError(target, current)

            // END CONDITION (2° tolerance)
            if (abs(error) < Math.toRadians(2.0)) break

            val pidOutput = pid.calculate(0.0, error)   // IMPORTANT

            val power = pidOutput.coerceIn(-maxPower, maxPower)

            setTurnPower(-power)

            Thread.sleep(10)
        }

        setTurnPower(0.0)
    }

    /**
     * Normalize angle to [-π, +π]
     */
    private fun normalizeRadians(angle: Double): Double =
        atan2(kotlin.math.sin(angle), kotlin.math.cos(angle))

    /**
     * Shortest signed angle difference in radians.
     */
    private fun angleError(target: Double, current: Double): Double {
        return normalizeRadians(target - current)
    }

    private fun setTurnPower(turn: Double) {
        (hardware["left_front"] as MotorEx).set(turn)
        (hardware["left_back"] as MotorEx).set(turn)
        (hardware["right_front"] as MotorEx).set(turn)
        (hardware["right_back"] as MotorEx).set(turn)
    }

    fun start() {
        limelight.start()
        odometry.resetPosAndIMU()
    }

    fun loop() {
        limelight.updateRobotOrientation(odometry.getHeading(AngleUnit.DEGREES))
        odometry.update()

        run()
    }
}