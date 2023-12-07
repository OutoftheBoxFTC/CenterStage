package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Vector2d

infix fun Vector2d.cross(other: Vector2d) = x * other.y - y * other.x
