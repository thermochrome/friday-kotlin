package org.firstinspires.ftc.teamcode.robot

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.gamepad.GamepadKeys

enum class Activation {
    TOGGLE, PRESS, HELD
}

data class ButtonData(
    val key: GamepadKeys.Button,
    val activated: Command,
    val deactivated: Command? = null
)