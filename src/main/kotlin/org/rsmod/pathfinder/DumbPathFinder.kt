@file:Suppress("NOTHING_TO_INLINE")

package org.rsmod.pathfinder

import org.rsmod.pathfinder.collision.CollisionStrategy
import org.rsmod.pathfinder.reach.ReachStrategy
import kotlin.math.abs
import kotlin.math.sign

@Deprecated(
    """
    Deprecated as the dumb pathfinder is not an actual thing, instead the same code that is used
    to interpolate the tiles of the checkpoints provided by the smart pathfinder is used to
    generate the dumb paths. It is highly advised to switch to that implementation and lazily yield the
    tiles rather than continuously generate the dumb paths, as the two could have differentiating
    results when considering other moving creatures that might be interfering with the paths.
"""
)
public class DumbPathFinder(
    private val flags: Array<IntArray?>,
    private val defaultFlag: Int,
    private val initialRouteCapacity: Int = 3,
    private val diagonalSafespot: Boolean = true
) : PathFinder {

    override fun findPath(
        srcX: Int,
        srcY: Int,
        destX: Int,
        destY: Int,
        z: Int,
        srcSize: Int,
        destWidth: Int,
        destHeight: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        maxTurns: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
        reachStrategy: ReachStrategy
    ): Route {
        var curX = srcX
        var curY = srcY
        var curDirection: Direction? = null
        val turns = ArrayDeque<RouteCoordinates>(initialRouteCapacity)
        val flags = this.flags
        val defaultFlag = this.defaultFlag
        while (true) {
            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    curX,
                    curY,
                    z,
                    destX,
                    destY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask
                )
            ) {
                /* If we didn't move at all, return an empty route. */
                if (curX == srcX && curY == srcY) return SUCCESSFUL_ROUTE
                /* Otherwise, append the destination coords, and return the route. */
                turns.addLast(RouteCoordinates(curX, curY))
                return Route(turns, alternative = false, success = true)
            }
            val direction = directionBetween(curX, curY, destX, destY)
            val horizontalDirection = direction.horizontalDirection
            /* If we can reach our target by taking the horizontal step out of the diagonal step alone, prefer that. */
            val exceptionRoute = exceptions(
                horizontalDirection,
                direction,
                curX,
                curY,
                reachStrategy,
                flags,
                defaultFlag,
                z,
                destX,
                destY,
                destWidth,
                destHeight,
                srcSize,
                objRot,
                objShape,
                accessBitMask,
                extraFlagToCheck,
                collision,
                curDirection,
                turns,
                maxTurns
            )
            if (exceptionRoute != null) return exceptionRoute
            val verticalDirection = direction.verticalDirection
            /* Pick one of the three potential directions, preferring diagonal -> horizontal -> vertical. */
            val moveDirection = when {
                !direction.isBlocked(
                    flags,
                    defaultFlag,
                    extraFlagToCheck,
                    curX,
                    curY,
                    z,
                    collision,
                    srcSize
                ) -> direction
                horizontalDirection != null && horizontalDirection != direction && !horizontalDirection.isBlocked(
                    flags,
                    defaultFlag,
                    extraFlagToCheck,
                    curX,
                    curY,
                    z,
                    collision,
                    srcSize
                ) -> horizontalDirection
                verticalDirection != null && verticalDirection != direction && !verticalDirection.isBlocked(
                    flags,
                    defaultFlag,
                    extraFlagToCheck,
                    curX,
                    curY,
                    z,
                    collision,
                    srcSize
                ) -> verticalDirection
                else -> break
            }
            if (curDirection == null) {
                curDirection = moveDirection
            } else if (curDirection != moveDirection) {
                if (turns.size >= maxTurns) turns.removeFirst()
                turns.addLast(RouteCoordinates(curX, curY))
                curDirection = moveDirection
            }
            curX += moveDirection.offX
            curY += moveDirection.offY
        }
        if (curX == srcX && curY == srcY) return FAILED_ROUTE
        turns.addLast(RouteCoordinates(curX, curY))
        return Route(turns, alternative = turns.isNotEmpty(), success = false)
    }

    private inline fun exceptions(
        horizontalDirection: Direction?,
        direction: Direction,
        curX: Int,
        curY: Int,
        reachStrategy: ReachStrategy,
        flags: Array<IntArray?>,
        defaultFlag: Int,
        z: Int,
        destX: Int,
        destY: Int,
        destWidth: Int,
        destHeight: Int,
        srcSize: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        entityCheckExtraFlags: Int,
        collision: CollisionStrategy,
        curDirection: Direction?,
        turns: ArrayDeque<RouteCoordinates>,
        maxTurns: Int
    ): Route? {
        if (horizontalDirection != null && direction != horizontalDirection) {
            val splitX = curX + horizontalDirection.offX
            val splitY = curY + horizontalDirection.offY
            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    splitX,
                    splitY,
                    z,
                    destX,
                    destY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask
                )
            ) {
                /* If horizontal step is not blocked(note: reached check is cheaper than block check), return the path here. */
                if (!horizontalDirection.isBlocked(
                        flags,
                        defaultFlag,
                        entityCheckExtraFlags,
                        curX,
                        curY,
                        z,
                        collision,
                        srcSize
                    )
                ) {
                    if (curDirection != null && curDirection != horizontalDirection) {
                        if (turns.size >= maxTurns) turns.removeFirst()
                        turns.addLast(RouteCoordinates(curX, curY))
                    }
                    if (turns.size >= maxTurns) turns.removeFirst()
                    turns.addLast(RouteCoordinates(splitX, splitY))
                    return Route(turns, alternative = false, success = true)
                } else if (diagonalSafespot &&
                    getAxisDistances(curX, curY, srcSize, srcSize, destX, destY, destWidth, destHeight).diagonal
                ) {
                    /* If diagonal safespot is enabled, and we are standing diagonally from our destination, return the path prematurely. */
                    if (curDirection != null && curDirection != horizontalDirection) {
                        if (turns.size >= maxTurns) turns.removeFirst()
                        turns.addLast(RouteCoordinates(curX, curY))
                    }
                    return Route(turns, alternative = turns.isNotEmpty(), success = false)
                }
            }
        }
        return null
    }

    private inline operator fun Array<IntArray?>.get(x: Int, y: Int, level: Int): Int {
        val zone = this[getZoneIndex(x, y, level)] ?: return defaultFlag
        return zone[getIndexInZone(x, y)]
    }

    private inline fun getZoneIndex(x: Int, y: Int, z: Int): Int =
        ((x shr 3) and 0x7FF) or (((y shr 3) and 0x7FF) shl 11) or ((z and 0x3) shl 22)

    private inline fun getIndexInZone(x: Int, y: Int): Int = (x and 0x7) or ((y and 0x7) shl 3)

    private inline fun directionBetween(srcX: Int, srcY: Int, destX: Int, destY: Int): Direction {
        val xOff = (destX - srcX).sign
        val yOff = (destY - srcY).sign
        return getDirection(xOff, yOff)
    }

    internal companion object {
        private val SUCCESSFUL_ROUTE = Route(ArrayDeque(), alternative = false, success = true)
        private val FAILED_ROUTE = Route(ArrayDeque(), alternative = false, success = false)
        private val allDirections = listOf(South, North, West, East, SouthWest, NorthWest, SouthEast, NorthEast)
        private val mappedDirections = List(0xF) { key ->
            allDirections.find { bitpackDirection(it.offX, it.offY) == key }
        }

        private inline fun bitpackDirection(xOff: Int, yOff: Int): Int = xOff.inc().shl(2) or yOff.inc()

        inline fun getDirection(xOff: Int, yOff: Int): Direction {
            assert(xOff in -1..1) { "X offset must be in bounds of -1..1" }
            assert(yOff in -1..1) { "Y offset must be in bounds of -1..1" }
            return mappedDirections[bitpackDirection(xOff, yOff)]
                ?: throw IllegalArgumentException("Offsets [$xOff, $yOff] do not produce a valid movement direction.")
        }
    }

    private inline fun getAxisDistances(
        x1: Int,
        y1: Int,
        width1: Int,
        height1: Int,
        x2: Int,
        y2: Int,
        width2: Int,
        height2: Int
    ): Point {
        val p1 = getComparisonPoint(x1, y1, width1, height1, x2, y2)
        val p2 = getComparisonPoint(x2, y2, width2, height2, x1, y1)
        return Point(abs(p1.x - p2.x), abs(p1.y - p2.y))
    }

    private inline fun getComparisonPoint(x1: Int, y1: Int, width1: Int, height1: Int, x2: Int, y2: Int): Point {
        val x = when {
            x2 <= x1 -> x1
            x2 >= x1 + width1 - 1 -> x1 + width1 - 1
            else -> x2
        }
        val y = when {
            y2 <= y1 -> y1
            y2 >= y1 + height1 - 1 -> y1 + height1 - 1
            else -> y2
        }
        return Point(x, y)
    }

    @JvmInline
    private value class Point(private val packed: Int) {
        constructor(x: Int, y: Int) : this((x and 0xFFFF) or ((y and 0xFFFF) shl 16))

        val x: Int get() = packed and 0xFFFF
        val y: Int get() = (packed shr 16) and 0xFFFF
        val diagonal: Boolean get() = packed == 0x10001
    }
}
