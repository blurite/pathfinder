package org.rsmod.pathfinder.bound

import org.rsmod.pathfinder.flag.AccessBitFlag
import org.rsmod.pathfinder.flag.CollisionFlag
import kotlin.math.max
import kotlin.math.min

/**
 * @author Kris | 12/09/2021
 */
public object RectangleBoundaryUtils {
    public fun collides(
        srcX: Int,
        srcY: Int,
        destX: Int,
        destY: Int,
        srcWidth: Int,
        srcHeight: Int,
        destWidth: Int,
        destHeight: Int
    ): Boolean = if (srcX >= destX + destWidth || srcX + srcWidth <= destX) {
        false
    } else {
        srcY < destY + destHeight && destY < srcHeight + srcY
    }

    internal fun reachRectangle1(
        flags: Array<IntArray?>,
        defaultFlag: Int,
        x: Int,
        y: Int,
        z: Int,
        accessBitMask: Int,
        destX: Int,
        destY: Int,
        destWidth: Int,
        destHeight: Int
    ): Boolean {
        val east = destX + destWidth - 1
        val north = destY + destHeight - 1

        if (x == destX - 1 && y >= destY && y <= north &&
            (flags[defaultFlag, x, y, z] and CollisionFlag.WALL_EAST) == 0 &&
            (accessBitMask and AccessBitFlag.BLOCK_WEST) == 0
        ) return true

        if (x == east + 1 && y >= destY && y <= north &&
            (flags[defaultFlag, x, y, z] and CollisionFlag.WALL_WEST) == 0 &&
            (accessBitMask and AccessBitFlag.BLOCK_EAST) == 0
        ) return true

        if (y + 1 == destY && x >= destX && x <= east &&
            (flags[defaultFlag, x, y, z] and CollisionFlag.WALL_NORTH) == 0 &&
            (accessBitMask and AccessBitFlag.BLOCK_SOUTH) == 0

        ) return true

        return y == north + 1 && x >= destX && x <= east &&
            (flags[defaultFlag, x, y, z] and CollisionFlag.WALL_SOUTH) == 0 &&
            (accessBitMask and AccessBitFlag.BLOCK_NORTH) == 0
    }

    internal fun reachRectangleN(
        flags: Array<IntArray?>,
        defaultFlag: Int,
        x: Int,
        y: Int,
        z: Int,
        accessBitMask: Int,
        destX: Int,
        destY: Int,
        srcWidth: Int,
        srcHeight: Int,
        destWidth: Int,
        destHeight: Int
    ): Boolean {
        val srcEast = x + srcWidth
        val srcNorth = srcHeight + y
        val destEast = destWidth + destX
        val destNorth = destHeight + destY
        if (destEast == x && (accessBitMask and AccessBitFlag.BLOCK_EAST) == 0) {
            val fromY = max(y, destY)
            val toY = min(srcNorth, destNorth)
            for (sideY in fromY until toY) {
                if (flags[defaultFlag, destEast - 1, sideY, z] and CollisionFlag.WALL_EAST == 0) {
                    return true
                }
            }
        } else if (srcEast == destX && (accessBitMask and AccessBitFlag.BLOCK_WEST) == 0) {
            val fromY = max(y, destY)
            val toY = min(srcNorth, destNorth)
            for (sideY in fromY until toY) {
                if (flags[defaultFlag, destX, sideY, z] and CollisionFlag.WALL_WEST == 0) {
                    return true
                }
            }
        } else if (y == destNorth && (accessBitMask and AccessBitFlag.BLOCK_NORTH) == 0) {
            val fromX = max(x, destX)
            val toX = min(srcEast, destEast)
            for (sideX in fromX until toX) {
                if (flags[defaultFlag, sideX, destNorth - 1, z] and CollisionFlag.WALL_NORTH == 0) {
                    return true
                }
            }
        } else if (destY == srcNorth && (accessBitMask and AccessBitFlag.BLOCK_SOUTH) == 0) {
            val fromX = max(x, destX)
            val toX = min(srcEast, destEast)
            for (sideX in fromX until toX) {
                if (flags[defaultFlag, sideX, destY, z] and CollisionFlag.WALL_SOUTH == 0) {
                    return true
                }
            }
        }
        return false
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline operator fun Array<IntArray?>.get(
        defaultFlag: Int,
        x: Int,
        y: Int,
        z: Int
    ): Int {
        val zone = this[getZoneIndex(x, y, z)] ?: return defaultFlag
        return zone[getIndexInZone(x, y)]
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline fun getZoneIndex(x: Int, y: Int, z: Int): Int {
        return ((x shr 3) and 0x7FF) or (((y shr 3) and 0x7FF) shl 11) or ((z and 0x3) shl 22)
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline fun getIndexInZone(x: Int, y: Int): Int {
        return (x and 0x7) or ((y and 0x7) shl 3)
    }
}
