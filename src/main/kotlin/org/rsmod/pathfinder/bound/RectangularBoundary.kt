package org.rsmod.pathfinder.bound

internal fun reachRectangle(
    flags: Array<IntArray?>,
    defaultFlag: Int,
    x: Int,
    y: Int,
    z: Int,
    accessBitMask: Int,
    destX: Int,
    destY: Int,
    srcSize: Int,
    destWidth: Int,
    destHeight: Int
): Boolean = when {
    srcSize > 1 -> {
        RectangleBoundaryUtils
            .collides(x, y, destX, destY, srcSize, srcSize, destWidth, destHeight) ||
            RectangleBoundaryUtils.reachRectangleN(
                flags,
                defaultFlag,
                x,
                y,
                z,
                accessBitMask,
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
            .collides(x, y, destX, destY, srcSize, srcSize, destWidth, destHeight) ||
            RectangleBoundaryUtils.reachRectangle1(
                flags,
                defaultFlag,
                x,
                y,
                z,
                accessBitMask,
                destX,
                destY,
                destWidth,
                destHeight
            )
}
