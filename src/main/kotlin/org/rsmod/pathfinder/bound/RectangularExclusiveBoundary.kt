package org.rsmod.pathfinder.bound

/**
 * @author Kris | 12/09/2021
 */
internal fun reachExclusiveRectangle(
    flags: IntArray,
    mapSize: Int,
    accessBitMask: Int,
    srcX: Int,
    srcY: Int,
    destX: Int,
    destY: Int,
    srcSize: Int,
    destWidth: Int,
    destHeight: Int
): Boolean = when {
    srcSize > 1 -> {
        if (RectangleBoundaryUtils.collides(srcX, srcY, destX, destY, srcSize, srcSize, destWidth, destHeight)) false
        else RectangleBoundaryUtils.reachRectangleN(
            flags,
            mapSize,
            accessBitMask,
            srcX,
            srcY,
            destX,
            destY,
            srcSize,
            srcSize,
            destWidth,
            destHeight
        )
    }
    else ->
        RectangleBoundaryUtils
            .reachRectangle1(flags, mapSize, accessBitMask, srcX, srcY, destX, destY, destWidth, destHeight)
}