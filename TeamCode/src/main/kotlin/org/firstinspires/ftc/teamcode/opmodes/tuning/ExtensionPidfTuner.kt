package org.firstinspires.ftc.teamcode.opmodes.tuning

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Subsystem
import org.firstinspires.ftc.teamcode.actions.hardware.extensionLength
import org.firstinspires.ftc.teamcode.actions.hardware.setExtensionPower

@TeleOp
class ExtensionPidfTuner : PidfTuner(Subsystem.EXTENSION) {
    override fun updateOutput(output: Double) = setExtensionPower(output)

    override fun readInput() = extensionLength().toDouble()
}