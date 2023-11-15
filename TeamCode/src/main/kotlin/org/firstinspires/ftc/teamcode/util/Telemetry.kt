package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.Telemetry

operator fun Telemetry.set(caption: String, value: Any) { addData(caption, value) }
