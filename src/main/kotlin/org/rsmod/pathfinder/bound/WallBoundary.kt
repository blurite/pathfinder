@file:Suppress("DuplicatedCode")

package org.rsmod.pathfinder.bound

import org.rsmod.pathfinder.flag.CollisionFlag

internal fun reachWall(
    flags: IntArray,
    mapSize: Int,
    localSrcX: Int,
    localSrcY: Int,
    localDestX: Int,
    localDestY: Int,
    srcSize: Int,
    shape: Int,
    rot: Int
): Boolean = when {
    srcSize == 1 && localSrcX == localDestX && localSrcY == localDestY -> true
    srcSize != 1 && localDestX >= localSrcX && srcSize + localSrcX - 1 >= localDestX &&
        srcSize + localDestY - 1 >= localDestY -> true
    srcSize == 1 -> reachWall1(flags, mapSize, localSrcX, localSrcY, localDestX, localDestY, shape, rot)
    else -> reachWallN(flags, mapSize, localSrcX, localSrcY, localDestX, localDestY, srcSize, shape, rot)
}

private fun reachWall1(
    flags: IntArray,
    mapSize: Int,
    localSrcX: Int,
    localSrcY: Int,
    localDestX: Int,
    localDestY: Int,
    shape: Int,
    rot: Int
): Boolean {
    when (shape) {
        0 -> {
            when (rot) {
                0 -> {
                    if (localSrcX == localDestX - 1 && localSrcY == localDestY)
                        return true
                    if (localSrcX == localDestX && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localSrcX == localDestX && localSrcY == localDestY - 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                1 -> {
                    if (localSrcX == localDestX && localSrcY == localDestY + 1)
                        return true
                    if (localSrcX == localDestX - 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                }
                2 -> {
                    if (localSrcX == localDestX + 1 && localSrcY == localDestY)
                        return true
                    if (localSrcX == localDestX && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localSrcX == localDestX && localSrcY == localDestY - 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                3 -> {
                    if (localSrcX == localDestX && localSrcY == localDestY - 1)
                        return true
                    if (localSrcX == localDestX - 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                }
            }
        }
        2 -> {
            when (rot) {
                0 -> {
                    if (localSrcX == localDestX - 1 && localSrcY == localDestY)
                        return true
                    if (localSrcX == localDestX && localSrcY == localDestY + 1)
                        return true
                    if (localSrcX == localDestX + 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                    if (localSrcX == localDestX && localSrcY == localDestY - 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                1 -> {
                    if (localSrcX == localDestX - 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localSrcX == localDestX && localSrcY == localDestY + 1)
                        return true
                    if (localSrcX == localDestX + 1 && localSrcY == localDestY)
                        return true
                    if (localSrcX == localDestX && localSrcY == localDestY - 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                2 -> {
                    if (localSrcX == localDestX - 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localSrcX == localDestX && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY == localDestY)
                        return true
                    if (localSrcX == localDestX && localSrcY == localDestY - 1)
                        return true
                }
                3 -> {
                    if (localSrcX == localDestX - 1 && localSrcY == localDestY)
                        return true
                    if (localSrcX == localDestX && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY == localDestY &&
                        (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                    if (localSrcX == localDestX && localSrcY == localDestY - 1)
                        return true
                }
            }
        }
        9 -> {
            if (localSrcX == localDestX && localSrcY == localDestY + 1 &&
                (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.WALL_SOUTH) == 0
            ) return true
            if (localSrcX == localDestX && localSrcY == localDestY - 1 &&
                (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.WALL_NORTH) == 0
            ) return true
            if (localSrcX == localDestX - 1 && localSrcY == localDestY &&
                (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.WALL_EAST) == 0
            ) return true

            return localSrcX == localDestX + 1 && localSrcY == localDestY &&
                (flag(flags, mapSize, localSrcX, localSrcY) and CollisionFlag.WALL_WEST) == 0
        }
    }
    return false
}

private fun reachWallN(
    flags: IntArray,
    mapSize: Int,
    localSrcX: Int,
    localSrcY: Int,
    localDestX: Int,
    localDestY: Int,
    srcSize: Int,
    shape: Int,
    rot: Int
): Boolean {
    val east = localSrcX + srcSize - 1
    val north = localSrcY + srcSize - 1
    when (shape) {
        0 -> {
            when (rot) {
                0 -> {
                    if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY)
                        return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localDestX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize &&
                        (flag(flags, mapSize, localDestX, north) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                1 -> {
                    if (localDestX in localSrcX..east && localSrcY == localDestY + 1)
                        return true
                    if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, east, localDestY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, localSrcX, localDestY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                }
                2 -> {
                    if (localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY)
                        return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localDestX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize &&
                        (flag(flags, mapSize, localDestX, north) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                3 -> {
                    if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize)
                        return true
                    if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, east, localDestY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, localSrcX, localDestY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                }
            }
        }
        2 -> {
            when (rot) {
                0 -> {
                    if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY)
                        return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY + 1)
                        return true
                    if (localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, localSrcX, localDestY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize &&
                        (flag(flags, mapSize, localDestX, north) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                1 -> {
                    if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, east, localDestY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY + 1)
                        return true
                    if (localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY)
                        return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize &&
                        (flag(flags, mapSize, localDestX, north) and CollisionFlag.BLOCK_SOUTH) == 0
                    ) return true
                }
                2 -> {
                    if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, east, localDestY) and CollisionFlag.BLOCK_WEST) == 0
                    ) return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localDestX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY)
                        return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize)
                        return true
                }
                3 -> {
                    if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY)
                        return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY + 1 &&
                        (flag(flags, mapSize, localDestX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
                    ) return true
                    if (localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY &&
                        (flag(flags, mapSize, localSrcX, localDestY) and CollisionFlag.BLOCK_EAST) == 0
                    ) return true
                    if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize)
                        return true
                }
            }
        }
        9 -> {
            if (localDestX in localSrcX..east && localSrcY == localDestY + 1 &&
                (flag(flags, mapSize, localDestX, localSrcY) and CollisionFlag.BLOCK_NORTH) == 0
            ) return true
            if (localDestX in localSrcX..east && localSrcY == localDestY - srcSize &&
                (flag(flags, mapSize, localDestX, north) and CollisionFlag.BLOCK_SOUTH) == 0
            ) return true
            if (localSrcX == localDestX - srcSize && localSrcY <= localDestY && north >= localDestY &&
                (flag(flags, mapSize, east, localDestY) and CollisionFlag.BLOCK_WEST) == 0
            ) return true

            return localSrcX == localDestX + 1 && localSrcY <= localDestY && north >= localDestY &&
                (flag(flags, mapSize, localSrcX, localDestY) and CollisionFlag.BLOCK_EAST) == 0
        }
    }
    return false
}

private fun flag(flags: IntArray, width: Int, x: Int, y: Int): Int {
    return flags[(y * width) + x]
}
