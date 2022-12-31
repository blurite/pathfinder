package org.rsmod.pathfinder

import org.rsmod.pathfinder.collision.CollisionStrategy

/**
 * @author Kris | 16/03/2022
 */
public class StepValidator(
    private val flags: Array<IntArray?>,
) {
    public fun canTravel(
        x: Int,
        y: Int,
        z: Int,
        size: Int,
        offsetX: Int,
        offsetY: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
    ): Boolean {
        assert(offsetX in -1..1) { "Offset x must be in bounds of -1..1" }
        assert(offsetY in -1..1) { "Offset y must be in bounds of -1..1" }
        assert(offsetX != 0 || offsetY != 0) { "Offset x and y cannot both be 0." }
        val direction = getDirection(offsetX, offsetY)
        return !direction.isBlocked(
            flags,
            extraFlagToCheck,
            x,
            y,
            z,
            collision,
            size
        )
    }

    internal companion object {
        private val allDirections = listOf(South, North, West, East, SouthWest, NorthWest, SouthEast, NorthEast)
        private val mappedDirections = List(0xF) { key ->
            allDirections.find { bitpackDirection(it.offX, it.offY) == key }
        }

        @Suppress("NOTHING_TO_INLINE")
        private inline fun bitpackDirection(xOff: Int, yOff: Int): Int = xOff.inc().shl(2) or yOff.inc()

        @Suppress("NOTHING_TO_INLINE")
        inline fun getDirection(xOff: Int, yOff: Int): Direction {
            assert(xOff in -1..1) { "X offset must be in bounds of -1..1" }
            assert(yOff in -1..1) { "Y offset must be in bounds of -1..1" }
            return mappedDirections[bitpackDirection(xOff, yOff)]
                ?: throw IllegalArgumentException("Offsets [$xOff, $yOff] do not produce a valid movement direction.")
        }
    }
}
