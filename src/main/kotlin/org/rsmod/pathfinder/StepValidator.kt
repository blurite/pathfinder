package org.rsmod.pathfinder

import org.rsmod.pathfinder.collision.CollisionStrategies
import org.rsmod.pathfinder.collision.CollisionStrategy
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_EAST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_NORTH
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_NORTH_EAST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_NORTH_EAST_AND_WEST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_NORTH_WEST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_SOUTH
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_SOUTH_EAST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_SOUTH_WEST
import org.rsmod.pathfinder.flag.CollisionFlag.BLOCK_WEST

/**
 * @author Kris | 16/03/2022
 */
public class StepValidator(private val flags: Array<IntArray?>) {

    public fun canTravel(
        level: Int,
        x: Int,
        y: Int,
        offsetX: Int,
        offsetY: Int,
        size: Int = 1,
        extraFlag: Int,
        collision: CollisionStrategy = CollisionStrategies.Normal,
    ): Boolean {
        val index = singleCellIndex(offsetX, offsetY)
        val blocked = when (singleCellMovementOpcodes[index]) {
            S -> isBlockedSouth(flags, level, x, y, size, extraFlag, collision)
            N -> isBlockedNorth(flags, level, x, y, size, extraFlag, collision)
            W -> isBlockedWest(flags, level, x, y, size, extraFlag, collision)
            E -> isBlockedEast(flags, level, x, y, size, extraFlag, collision)
            SW -> isBlockedSouthWest(flags, level, x, y, size, extraFlag, collision)
            NW -> isBlockedNorthWest(flags, level, x, y, size, extraFlag, collision)
            SE -> isBlockedSouthEast(flags, level, x, y, size, extraFlag, collision)
            NE -> isBlockedNorthEast(flags, level, x, y, size, extraFlag, collision)
            else -> error("Invalid offsets: $offsetX, $offsetY")
        }
        return !blocked
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline operator fun Array<IntArray?>.get(x: Int, y: Int, level: Int): Int {
        val zone = this[getZoneIndex(x, y, level)] ?: return -1
        return zone[getIndexInZone(x, y)]
    }

    private fun isBlockedSouth(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x, y - 1, level], BLOCK_SOUTH or extraFlag)
            2 -> !collision.canMove(flags[x, y - 1, level], BLOCK_SOUTH_WEST or extraFlag) ||
                !collision.canMove(flags[x + 1, y - 1, level], BLOCK_SOUTH_EAST or extraFlag)
            else -> {
                if (
                    !collision.canMove(flags[x, y - 1, level], BLOCK_SOUTH_WEST or extraFlag) ||
                    !collision.canMove(flags[x + size - 1, y - 1, level], BLOCK_SOUTH_EAST or extraFlag)
                ) {
                    return true
                }
                for (midX in x + 1 until x + size - 1) {
                    if (!collision.canMove(
                            flags[midX, y - 1, level],
                            BLOCK_NORTH_EAST_AND_WEST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private fun isBlockedNorth(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x, y + 1, level], BLOCK_NORTH or extraFlag)
            2 -> !collision.canMove(flags[x, y + 2, level], BLOCK_NORTH_WEST or extraFlag) ||
                !collision.canMove(flags[x + 1, y + 2, level], BLOCK_NORTH_EAST or extraFlag)
            else -> {
                if (
                    !collision.canMove(flags[x, y + size, level], BLOCK_NORTH_WEST or extraFlag) ||
                    !collision.canMove(
                        flags[x + size - 1, y + size, level],
                        BLOCK_NORTH_EAST or extraFlag
                    )
                ) {
                    return true
                }
                for (midX in x + 1 until x + size - 1) {
                    if (!collision.canMove(
                            flags[midX, y + size, level],
                            BLOCK_SOUTH_EAST_AND_WEST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private fun isBlockedWest(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x - 1, y, level], BLOCK_WEST or extraFlag)
            2 -> !collision.canMove(flags[x - 1, y, level], BLOCK_SOUTH_WEST or extraFlag) ||
                !collision.canMove(flags[x - 1, y + 1, level], BLOCK_NORTH_WEST or extraFlag)
            else -> {
                if (
                    !collision.canMove(flags[x - 1, y, level], BLOCK_SOUTH_WEST or extraFlag) ||
                    !collision.canMove(flags[x - 1, y + size - 1, level], BLOCK_NORTH_WEST or extraFlag)
                ) {
                    return true
                }
                for (midY in y + 1 until y + size - 1) {
                    if (!collision.canMove(
                            flags[x - 1, midY, level],
                            BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private fun isBlockedEast(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x + 1, y, level], BLOCK_EAST or extraFlag)
            2 -> !collision.canMove(flags[x + 2, y, level], BLOCK_SOUTH_EAST or extraFlag) ||
                !collision.canMove(flags[x + 2, y + 1, level], BLOCK_NORTH_EAST or extraFlag)
            else -> {
                if (
                    !collision.canMove(flags[x + size, y, level], BLOCK_SOUTH_EAST or extraFlag) ||
                    !collision.canMove(
                        flags[x + size, y + size - 1, level],
                        BLOCK_NORTH_EAST or extraFlag
                    )
                ) {
                    return true
                }
                for (midY in y + 1 until y + size - 1) {
                    if (!collision.canMove(
                            flags[x + size, midY, level],
                            BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private fun isBlockedSouthWest(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x - 1, y - 1, level], BLOCK_SOUTH_WEST or extraFlag) ||
                !collision.canMove(flags[x - 1, y, level], BLOCK_WEST or extraFlag) ||
                !collision.canMove(flags[x, y - 1, level], BLOCK_SOUTH or extraFlag)
            2 -> !collision.canMove(flags[x - 1, y, level], BLOCK_NORTH_AND_SOUTH_EAST or extraFlag) ||
                !collision.canMove(flags[x - 1, y - 1, level], BLOCK_SOUTH_WEST or extraFlag) ||
                !collision.canMove(flags[x, y - 1, level], BLOCK_NORTH_EAST_AND_WEST or extraFlag)
            else -> {
                if (!collision.canMove(flags[x - 1, y - 1, level], BLOCK_SOUTH_WEST or extraFlag)) {
                    return true
                }
                for (mid in 1 until size) {
                    if (
                        !collision.canMove(
                            flags[x - 1, y + mid - 1, level],
                            BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
                        ) ||
                        !collision.canMove(
                            flags[x + mid - 1, y - 1, level],
                            BLOCK_NORTH_EAST_AND_WEST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private fun isBlockedNorthWest(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x - 1, y + 1, level], BLOCK_NORTH_WEST or extraFlag) ||
                !collision.canMove(flags[x - 1, y, level], BLOCK_WEST or extraFlag) ||
                !collision.canMove(flags[x, y + 1, level], BLOCK_NORTH or extraFlag)
            2 -> !collision.canMove(flags[x - 1, y + 1, level], BLOCK_NORTH_AND_SOUTH_EAST or extraFlag) ||
                !collision.canMove(flags[x - 1, y + 2, level], BLOCK_NORTH_WEST or extraFlag) ||
                !collision.canMove(flags[x, y + 2, level], BLOCK_SOUTH_EAST_AND_WEST or extraFlag)
            else -> {
                if (!collision.canMove(flags[x - 1, y + size, level], BLOCK_NORTH_WEST or extraFlag)) {
                    return true
                }
                for (mid in 1 until size) {
                    if (
                        !collision.canMove(
                            flags[x - 1, y + mid, level],
                            BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
                        ) ||
                        !collision.canMove(
                            flags[x + mid - 1, y + size, level],
                            BLOCK_SOUTH_EAST_AND_WEST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private fun isBlockedSouthEast(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x + 1, y - 1, level], BLOCK_SOUTH_EAST or extraFlag) ||
                !collision.canMove(flags[x + 1, y, level], BLOCK_EAST or extraFlag) ||
                !collision.canMove(flags[x, y - 1, level], BLOCK_SOUTH or extraFlag)
            2 -> !collision.canMove(flags[x + 1, y - 1, level], BLOCK_NORTH_EAST_AND_WEST or extraFlag) ||
                !collision.canMove(flags[x + 2, y - 1, level], BLOCK_SOUTH_EAST or extraFlag) ||
                !collision.canMove(flags[x + 2, y, level], BLOCK_NORTH_AND_SOUTH_WEST or extraFlag)
            else -> {
                if (!collision.canMove(
                        flags[x + size, y - 1, level],
                        BLOCK_SOUTH_EAST or extraFlag
                    )
                ) return true
                for (mid in 1 until size) {
                    if (
                        !collision.canMove(
                            flags[x + size, y + mid - 1, level],
                            BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
                        ) ||
                        !collision.canMove(
                            flags[x + mid, y - 1, level],
                            BLOCK_NORTH_EAST_AND_WEST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private fun isBlockedNorthEast(
        flags: Array<IntArray?>,
        level: Int,
        x: Int,
        y: Int,
        size: Int,
        extraFlag: Int,
        collision: CollisionStrategy
    ): Boolean {
        return when (size) {
            1 -> !collision.canMove(flags[x + 1, y + 1, level], BLOCK_NORTH_EAST or extraFlag) ||
                !collision.canMove(flags[x + 1, y, level], BLOCK_EAST or extraFlag) ||
                !collision.canMove(flags[x, y + 1, level], BLOCK_NORTH or extraFlag)
            2 -> !collision.canMove(flags[x + 1, y + 2, level], BLOCK_SOUTH_EAST_AND_WEST or extraFlag) ||
                !collision.canMove(flags[x + 2, y + 2, level], BLOCK_NORTH_EAST or extraFlag) ||
                !collision.canMove(flags[x + 2, y + 1, level], BLOCK_NORTH_AND_SOUTH_WEST or extraFlag)
            else -> {
                if (!collision.canMove(flags[x + size, y + size, level], BLOCK_NORTH_EAST or extraFlag)) {
                    return true
                }
                for (mid in 1 until size) {
                    if (
                        !collision.canMove(
                            flags[x + mid, y + size, level],
                            BLOCK_SOUTH_EAST_AND_WEST or extraFlag
                        ) ||
                        !collision.canMove(
                            flags[x + size, y + mid, level],
                            BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
                        )
                    ) {
                        return true
                    }
                }
                return false
            }
        }
    }

    private companion object {
        private const val NW: Int = 0
        private const val N: Int = 1
        private const val NE: Int = 2
        private const val W: Int = 3
        private const val E: Int = 4
        private const val SW: Int = 5
        private const val S: Int = 6
        private const val SE: Int = 7


        /**
         * Single cell movement opcodes in a len-16 array.
         */
        private val singleCellMovementOpcodes: IntArray = buildSingleCellMovementOpcodes()


        /**
         * Builds a simple bitpacked array of the bit codes for all the possible deltas.
         * This is simply a more efficient variant of the normal if-else chain of checking
         * the different delta combinations, as we are skipping a lot of branch prediction.
         * In a benchmark, the results showed ~603% increased performance.
         */
        private fun buildSingleCellMovementOpcodes(): IntArray {
            val array = IntArray(16) { -1 }
            array[singleCellIndex(-1, -1)] = SW
            array[singleCellIndex(0, -1)] = S
            array[singleCellIndex(1, -1)] = SE
            array[singleCellIndex(-1, 0)] = W
            array[singleCellIndex(1, 0)] = E
            array[singleCellIndex(-1, 1)] = NW
            array[singleCellIndex(0, 1)] = N
            array[singleCellIndex(1, 1)] = NE
            return array
        }

        /**
         * Gets the index for a single cell movement opcode based on the deltas,
         * where the deltas are expected to be either -1, 0 or 1.
         * @param deltaX the x-coordinate delta
         * @param deltaZ the z-coordinate delta
         * @return the index of the single cell opcode stored in [singleCellMovementOpcodes]
         */
        private fun singleCellIndex(
            deltaX: Int,
            deltaZ: Int,
        ): Int = (deltaX + 1).or((deltaZ + 1) shl 2)
    }
}
