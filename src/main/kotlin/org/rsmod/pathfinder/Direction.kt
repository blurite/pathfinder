package org.rsmod.pathfinder

import org.rsmod.pathfinder.collision.CollisionStrategy
import org.rsmod.pathfinder.flag.CollisionFlag

/**
 * @author Kris | 15/03/2022
 */
@Suppress("NOTHING_TO_INLINE")
internal sealed class Direction(
    val offX: Int,
    val offY: Int,
) {
    abstract val horizontalDirection: Direction?
    abstract val verticalDirection: Direction?

    inline fun isBlocked(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy,
        size: Int
    ): Boolean = when (size) {
        1 -> isBlocked1(flags, extraFlag, x, y, level, collisionStrategy)
        2 -> isBlocked2(flags, extraFlag, x, y, level, collisionStrategy)
        else -> isBlockedN(flags, extraFlag, x, y, level, size, collisionStrategy)
    }

    abstract fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean

    abstract fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean

    abstract fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean

    inline operator fun Array<IntArray?>.get(x: Int, y: Int, level: Int): Int {
        val zone = this[getZoneIndex(x, y, level)] ?: return 0
        return zone[getIndexInZone(x, y)]
    }

    inline fun getZoneIndex(x: Int, y: Int, z: Int): Int =
        ((x shr 3) and 0x7FF) or (((y shr 3) and 0x7FF) shl 11) or ((z and 0x3) shl 22)

    inline fun getIndexInZone(x: Int, y: Int): Int = (x and 0x7) or ((y and 0x7) shl 3)
}

internal object South : Direction(0, -1) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean = !collisionStrategy.canMove(flags[x, y - 1, level], CollisionFlag.BLOCK_SOUTH or extraFlag)

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean =
        !collisionStrategy.canMove(flags[x, y - 1, level], CollisionFlag.BLOCK_SOUTH_WEST or extraFlag) ||
            !collisionStrategy.canMove(
                flags[x + 1, y - 1, level],
                CollisionFlag.BLOCK_SOUTH_EAST or extraFlag
            )

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x, y - 1, level],
                CollisionFlag.BLOCK_SOUTH_WEST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                    flags[x + size - 1, y - 1, level],
                    CollisionFlag.BLOCK_SOUTH_EAST or extraFlag
                )
        ) return true
        for (midX in x + 1 until x + size - 1) {
            if (!collisionStrategy.canMove(
                    flags[midX, y - 1, level],
                    CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlag
                )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction? = null
    override val verticalDirection: Direction = South
}

internal object North : Direction(0, 1) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean = !collisionStrategy.canMove(flags[x, y + 1, level], CollisionFlag.BLOCK_NORTH or extraFlag)

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean =
        !collisionStrategy.canMove(flags[x, y + 2, level], CollisionFlag.BLOCK_NORTH_WEST or extraFlag) ||
            !collisionStrategy.canMove(
                flags[x + 1, y + 2, level],
                CollisionFlag.BLOCK_NORTH_EAST or extraFlag
            )

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x, y + size, level],
                CollisionFlag.BLOCK_NORTH_WEST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                    flags[x + size - 1, y + size, level],
                    CollisionFlag.BLOCK_NORTH_EAST or extraFlag
                )
        ) return true
        for (midX in x + 1 until x + size - 1) {
            if (!collisionStrategy.canMove(
                    flags[midX, y + size, level],
                    CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlag
                )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction? = null
    override val verticalDirection: Direction = North
}

internal object West : Direction(-1, 0) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean = !collisionStrategy.canMove(flags[x - 1, y, level], CollisionFlag.BLOCK_WEST or extraFlag)

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean =
        !collisionStrategy.canMove(flags[x - 1, y, level], CollisionFlag.BLOCK_SOUTH_WEST or extraFlag) ||
            !collisionStrategy.canMove(
                flags[x - 1, y + 1, level],
                CollisionFlag.BLOCK_NORTH_WEST or extraFlag
            )

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x - 1, y, level],
                CollisionFlag.BLOCK_SOUTH_WEST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                    flags[x - 1, y + size - 1, level],
                    CollisionFlag.BLOCK_NORTH_WEST or extraFlag
                )
        ) return true
        for (midY in y + 1 until y + size - 1) {
            if (!collisionStrategy.canMove(
                    flags[x - 1, midY, level],
                    CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
                )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction = West
    override val verticalDirection: Direction? = null
}

internal object East : Direction(1, 0) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean = !collisionStrategy.canMove(flags[x + 1, y, level], CollisionFlag.BLOCK_EAST or extraFlag)

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean =
        !collisionStrategy.canMove(flags[x + 2, y, level], CollisionFlag.BLOCK_SOUTH_EAST or extraFlag) ||
            !collisionStrategy.canMove(
                flags[x + 2, y + 1, level],
                CollisionFlag.BLOCK_NORTH_EAST or extraFlag
            )

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x + size, y, level],
                CollisionFlag.BLOCK_SOUTH_EAST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                    flags[x + size, y + size - 1, level],
                    CollisionFlag.BLOCK_NORTH_EAST or extraFlag
                )
        ) return true
        for (midY in y + 1 until y + size - 1) {
            if (!collisionStrategy.canMove(
                    flags[x + size, midY, level],
                    CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
                )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction = East
    override val verticalDirection: Direction? = null
}

internal object NorthEast : Direction(1, 1) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x + 1, y + 1, level],
            CollisionFlag.BLOCK_NORTH_EAST or extraFlag
        ) ||
            !collisionStrategy.canMove(flags[x + 1, y, level], CollisionFlag.BLOCK_EAST or extraFlag) ||
            !collisionStrategy.canMove(flags[x, y + 1, level], CollisionFlag.BLOCK_NORTH or extraFlag)
    }

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x + 1, y + 2, level],
            CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlag
        ) ||
            !collisionStrategy.canMove(
                flags[x + 2, y + 2, level],
                CollisionFlag.BLOCK_NORTH_EAST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                flags[x + 2, y + 1, level],
                CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
            )
    }

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x + size, y + size, level],
                CollisionFlag.BLOCK_NORTH_EAST or extraFlag
            )
        ) return true
        for (mid in 1 until size) {
            if (!collisionStrategy.canMove(
                    flags[x + mid, y + size, level],
                    CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlag
                ) ||
                !collisionStrategy.canMove(
                        flags[x + size, y + mid, level],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
                    )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction = East
    override val verticalDirection: Direction = North
}

internal object SouthEast : Direction(1, -1) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x + 1, y - 1, level],
            CollisionFlag.BLOCK_SOUTH_EAST or extraFlag
        ) ||
            !collisionStrategy.canMove(flags[x + 1, y, level], CollisionFlag.BLOCK_EAST or extraFlag) ||
            !collisionStrategy.canMove(flags[x, y - 1, level], CollisionFlag.BLOCK_SOUTH or extraFlag)
    }

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x + 1, y - 1, level],
            CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlag
        ) ||
            !collisionStrategy.canMove(
                flags[x + 2, y - 1, level],
                CollisionFlag.BLOCK_SOUTH_EAST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                flags[x + 2, y, level],
                CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
            )
    }

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x + size, y - 1, level],
                CollisionFlag.BLOCK_SOUTH_EAST or extraFlag
            )
        ) return true
        for (mid in 1 until size) {
            if (!collisionStrategy.canMove(
                    flags[x + size, y + mid - 1, level],
                    CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlag
                ) ||
                !collisionStrategy.canMove(
                        flags[x + mid, y - 1, level],
                        CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlag
                    )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction = East
    override val verticalDirection: Direction = South
}

internal object NorthWest : Direction(-1, 1) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x - 1, y + 1, level],
            CollisionFlag.BLOCK_NORTH_WEST or extraFlag
        ) ||
            !collisionStrategy.canMove(flags[x - 1, y, level], CollisionFlag.BLOCK_WEST or extraFlag) ||
            !collisionStrategy.canMove(flags[x, y + 1, level], CollisionFlag.BLOCK_NORTH or extraFlag)
    }

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x - 1, y + 1, level],
            CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
        ) ||
            !collisionStrategy.canMove(
                flags[x - 1, y + 2, level],
                CollisionFlag.BLOCK_NORTH_WEST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                flags[x, y + 2, level],
                CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlag
            )
    }

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x - 1, y + size, level],
                CollisionFlag.BLOCK_NORTH_WEST or extraFlag
            )
        ) return true
        for (mid in 1 until size) {
            if (!collisionStrategy.canMove(
                    flags[x - 1, y + mid, level],
                    CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
                ) ||
                !collisionStrategy.canMove(
                        flags[x + mid - 1, y + size, level],
                        CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlag
                    )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction = West
    override val verticalDirection: Direction = North
}

internal object SouthWest : Direction(-1, -1) {
    override fun isBlocked1(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x - 1, y - 1, level],
            CollisionFlag.BLOCK_SOUTH_WEST or extraFlag
        ) ||
            !collisionStrategy.canMove(flags[x - 1, y, level], CollisionFlag.BLOCK_WEST or extraFlag) ||
            !collisionStrategy.canMove(flags[x, y - 1, level], CollisionFlag.BLOCK_SOUTH or extraFlag)
    }

    override fun isBlocked2(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        return !collisionStrategy.canMove(
            flags[x - 1, y, level],
            CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
        ) ||
            !collisionStrategy.canMove(
                flags[x - 1, y - 1, level],
                CollisionFlag.BLOCK_SOUTH_WEST or extraFlag
            ) ||
            !collisionStrategy.canMove(
                flags[x, y - 1, level],
                CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlag
            )
    }

    override fun isBlockedN(
        flags: Array<IntArray?>,
        extraFlag: Int,
        x: Int,
        y: Int,
        level: Int,
        size: Int,
        collisionStrategy: CollisionStrategy
    ): Boolean {
        if (!collisionStrategy.canMove(
                flags[x - 1, y - 1, level],
                CollisionFlag.BLOCK_SOUTH_WEST or extraFlag
            )
        ) return true
        for (mid in 1 until size) {
            if (!collisionStrategy.canMove(
                    flags[x - 1, y + mid - 1, level],
                    CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlag
                ) ||
                !collisionStrategy.canMove(
                        flags[x + mid - 1, y - 1, level],
                        CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlag
                    )
            ) {
                return true
            }
        }
        return false
    }

    override val horizontalDirection: Direction = West
    override val verticalDirection: Direction = South
}
