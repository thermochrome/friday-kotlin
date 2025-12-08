package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.command.Subsystem
import com.arcrobotics.ftclib.drivebase.MecanumDrive
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import kotlin.math.abs
import kotlin.math.max

class Drive(hardware: Hardware): Subsystem {
    val drive = MecanumDrive(
        hardware["left_front"] as MotorEx,
        hardware["right_front"] as MotorEx,
        hardware["left_back"] as MotorEx,
        hardware["right_back"] as MotorEx
    )

    fun drive(gamepad: GamepadEx, power: Double) {
        val normalization = max(
            abs(gamepad.leftX) + abs(gamepad.leftY) + abs(gamepad.rightX),
            1.0
        )

        val x = (-gamepad.leftX / normalization) * power
        val y = (gamepad.leftY / normalization) * power
        val rotation = (gamepad.rightX / normalization) * power

        drive.driveRobotCentric(x, y, rotation)
    }
}