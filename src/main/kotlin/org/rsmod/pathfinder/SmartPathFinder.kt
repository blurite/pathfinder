@file:Suppress("DuplicatedCode")

package org.rsmod.pathfinder

import org.rsmod.pathfinder.collision.CollisionStrategy
import org.rsmod.pathfinder.flag.CollisionFlag
import org.rsmod.pathfinder.flag.DirectionFlag
import org.rsmod.pathfinder.reach.ReachStrategy

private const val DEFAULT_RESET_ON_SEARCH = true
internal const val DEFAULT_SEARCH_MAP_SIZE = 128
private const val DEFAULT_RING_BUFFER_SIZE = 4096
private const val DEFAULT_DISTANCE_VALUE = 99_999_999
private const val DEFAULT_SRC_DIRECTION_VALUE = 99
private const val MAX_ALTERNATIVE_ROUTE_LOWEST_COST = 1000
private const val MAX_ALTERNATIVE_ROUTE_SEEK_RANGE = 100
private const val MAX_ALTERNATIVE_ROUTE_DISTANCE_FROM_DESTINATION = 10
private const val DEFAULT_MOVE_NEAR_FLAG = true
private const val DEFAULT_ROUTE_BLOCKER_FLAGS = false
private const val INITIAL_DEQUE_SIZE = 25

public class SmartPathFinder(
    private val resetOnSearch: Boolean = DEFAULT_RESET_ON_SEARCH,
    private val searchMapSize: Int = DEFAULT_SEARCH_MAP_SIZE,
    private val ringBufferSize: Int = DEFAULT_RING_BUFFER_SIZE,
    private val graphInfo: IntArray = IntArray(searchMapSize * searchMapSize),
    private val validLocalX: IntArray = IntArray(ringBufferSize),
    private val validLocalY: IntArray = IntArray(ringBufferSize),
    private var bufReaderIndex: Int = 0,
    private var bufWriterIndex: Int = 0,
    private var currLocalX: Int = 0,
    private var currLocalY: Int = 0,
    private val useRouteBlockerFlags: Boolean = DEFAULT_ROUTE_BLOCKER_FLAGS,
    private val flags: Array<IntArray?>,
    private val defaultFlag: Int,
    private val moveNear: Boolean = DEFAULT_MOVE_NEAR_FLAG,
    private val initialDequeSize: Int = INITIAL_DEQUE_SIZE,
) : PathFinder {

    public override fun findPath(
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
        if (resetOnSearch) {
            reset()
        }
        val baseX = srcX - (searchMapSize / 2)
        val baseY = srcY - (searchMapSize / 2)
        val localSrcX = srcX - baseX
        val localSrcY = srcY - baseY
        val localDestX = destX - baseX
        val localDestY = destY - baseY
        setNextValidLocalCoords(localSrcX, localSrcY, DEFAULT_SRC_DIRECTION_VALUE, 0)
        val pathFound =
            if (useRouteBlockerFlags) {
                when (srcSize) {
                    1 -> findPath1RouteBlocker(
                        baseX,
                        baseY,
                        z,
                        localDestX,
                        localDestY,
                        destWidth,
                        destHeight,
                        srcSize,
                        objRot,
                        objShape,
                        accessBitMask,
                        extraFlagToCheck,
                        collision,
                        reachStrategy
                    )

                    2 -> findPath2RouteBlocker(
                        baseX,
                        baseY,
                        z,
                        localDestX,
                        localDestY,
                        destWidth,
                        destHeight,
                        srcSize,
                        objRot,
                        objShape,
                        accessBitMask,
                        extraFlagToCheck,
                        collision,
                        reachStrategy
                    )

                    else -> findPathNRouteBlocker(
                        baseX,
                        baseY,
                        z,
                        localDestX,
                        localDestY,
                        destWidth,
                        destHeight,
                        srcSize,
                        objRot,
                        objShape,
                        accessBitMask,
                        extraFlagToCheck,
                        collision,
                        reachStrategy
                    )
                }
            } else {
                when (srcSize) {
                    1 -> findPath1(
                        baseX,
                        baseY,
                        z,
                        localDestX,
                        localDestY,
                        destWidth,
                        destHeight,
                        srcSize,
                        objRot,
                        objShape,
                        accessBitMask,
                        extraFlagToCheck,
                        collision,
                        reachStrategy
                    )

                    2 -> findPath2(
                        baseX,
                        baseY,
                        z,
                        localDestX,
                        localDestY,
                        destWidth,
                        destHeight,
                        srcSize,
                        objRot,
                        objShape,
                        accessBitMask,
                        extraFlagToCheck,
                        collision,
                        reachStrategy
                    )

                    else -> findPathN(
                        baseX,
                        baseY,
                        z,
                        localDestX,
                        localDestY,
                        destWidth,
                        destHeight,
                        srcSize,
                        objRot,
                        objShape,
                        accessBitMask,
                        extraFlagToCheck,
                        collision,
                        reachStrategy
                    )
                }
            }
        if (!pathFound) {
            if (!moveNear) {
                return FAILED_ROUTE
            } else if (!findClosestApproachPoint(localSrcX, localSrcY, localDestX, localDestY, destWidth, destHeight)) {
                return FAILED_ROUTE
            }
        }
        val coordinates = ArrayDeque<RouteCoordinates>(initialDequeSize)
        var nextDir = getDirection(currLocalX, currLocalY)
        var currDir = -1
        var turns = 0
        for (i in 0 until searchMapSize * searchMapSize) {
            if (currLocalX == localSrcX && currLocalY == localSrcY) break
            if (currDir != nextDir) {
                turns++
                if (coordinates.size >= maxTurns) coordinates.removeLast()
                val coords = RouteCoordinates(currLocalX + baseX, currLocalY + baseY)
                coordinates.addFirst(coords)
                currDir = nextDir
            }
            if ((currDir and DirectionFlag.EAST) != 0) {
                currLocalX++
            } else if ((currDir and DirectionFlag.WEST) != 0) {
                currLocalX--
            }
            if ((currDir and DirectionFlag.NORTH) != 0) {
                currLocalY++
            } else if ((currDir and DirectionFlag.SOUTH) != 0) {
                currLocalY--
            }
            nextDir = getDirection(currLocalX, currLocalY)
        }
        return Route(coordinates, alternative = !pathFound, success = true)
    }

    private fun findPath1(
        baseX: Int,
        baseY: Int,
        z: Int,
        localDestX: Int,
        localDestY: Int,
        destWidth: Int,
        destHeight: Int,
        srcSize: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
        reachStrategy: ReachStrategy
    ): Boolean {
        var x: Int
        var y: Int
        var clipFlag: Int
        var dirFlag: Int
        val relativeSearchSize = searchMapSize - 1
        while (bufWriterIndex != bufReaderIndex) {
            currLocalX = validLocalX[bufReaderIndex]
            currLocalY = validLocalY[bufReaderIndex]
            bufReaderIndex = (bufReaderIndex + 1) and (ringBufferSize - 1)

            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    currLocalX + baseX,
                    currLocalY + baseY,
                    z,
                    localDestX + baseX,
                    localDestY + baseY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask,
                )
            ) {
                return true
            }

            val nextDistance = getDistance(currLocalX, currLocalY) + 1

            /* east to west */
            x = currLocalX - 1
            y = currLocalY
            clipFlag = CollisionFlag.BLOCK_WEST or extraFlagToCheck
            dirFlag = DirectionFlag.EAST
            if (currLocalX > 0 && getDirection(x, y) == 0 && collision.canMove(
                    flags[baseX, baseY, x, y, z],
                    clipFlag
                )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* west to east */
            x = currLocalX + 1
            y = currLocalY
            clipFlag = CollisionFlag.BLOCK_EAST or extraFlagToCheck
            dirFlag = DirectionFlag.WEST
            if (currLocalX < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], clipFlag)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north to south  */
            x = currLocalX
            y = currLocalY - 1
            clipFlag = CollisionFlag.BLOCK_SOUTH or extraFlagToCheck
            dirFlag = DirectionFlag.NORTH
            if (currLocalY > 0 && getDirection(x, y) == 0 && collision.canMove(
                    flags[baseX, baseY, x, y, z],
                    clipFlag
                )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south to north */
            x = currLocalX
            y = currLocalY + 1
            clipFlag = CollisionFlag.BLOCK_NORTH or extraFlagToCheck
            dirFlag = DirectionFlag.SOUTH
            if (currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], clipFlag)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-east to south-west */
            x = currLocalX - 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_EAST
            if (currLocalX > 0 && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_WEST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_WEST or extraFlagToCheck
                    ) &&
                collision.canMove(flags[baseX, baseY, currLocalX, y, z], CollisionFlag.BLOCK_SOUTH or extraFlagToCheck)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-west to south-east */
            x = currLocalX + 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_EAST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(flags[baseX, baseY, currLocalX, y, z], CollisionFlag.BLOCK_SOUTH or extraFlagToCheck)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-east to north-west */
            x = currLocalX - 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_EAST
            if (currLocalX > 0 && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_NORTH_WEST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_WEST or extraFlagToCheck
                    ) &&
                collision.canMove(flags[baseX, baseY, currLocalX, y, z], CollisionFlag.BLOCK_NORTH or extraFlagToCheck)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-west to north-east */
            x = currLocalX + 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_NORTH_EAST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(flags[baseX, baseY, currLocalX, y, z], CollisionFlag.BLOCK_NORTH or extraFlagToCheck)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }
        }
        return false
    }

    private fun findPath2(
        baseX: Int,
        baseY: Int,
        z: Int,
        localDestX: Int,
        localDestY: Int,
        destWidth: Int,
        destHeight: Int,
        srcSize: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
        reachStrategy: ReachStrategy
    ): Boolean {
        var x: Int
        var y: Int
        var dirFlag: Int
        val relativeSearchSize = searchMapSize - 2
        while (bufWriterIndex != bufReaderIndex) {
            currLocalX = validLocalX[bufReaderIndex]
            currLocalY = validLocalY[bufReaderIndex]
            bufReaderIndex = (bufReaderIndex + 1) and (ringBufferSize - 1)

            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    currLocalX + baseX,
                    currLocalY + baseY,
                    z,
                    localDestX + baseX,
                    localDestY + baseY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask,
                )
            ) {
                return true
            }

            val nextDistance = getDistance(currLocalX, currLocalY) + 1

            /* east to west */
            x = currLocalX - 1
            y = currLocalY
            dirFlag = DirectionFlag.EAST
            if (currLocalX > 0 && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_WEST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 1, z],
                        CollisionFlag.BLOCK_NORTH_WEST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* west to east */
            x = currLocalX + 1
            y = currLocalY
            dirFlag = DirectionFlag.WEST
            if (currLocalX < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, currLocalY + 1, z],
                        CollisionFlag.BLOCK_NORTH_EAST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north to south  */
            x = currLocalX
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH
            if (currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_WEST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 1, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south to north */
            x = currLocalX
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH
            if (currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_WEST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 1, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_EAST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-east to south-west */
            x = currLocalX - 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_EAST
            if (currLocalX > 0 && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_WEST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, y, z],
                        CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-west to south-east */
            x = currLocalX + 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, currLocalY, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-east to north-west */
            x = currLocalX - 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_EAST
            if (currLocalX > 0 && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_WEST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, currLocalY + 2, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-west to north-east */
            x = currLocalX + 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 2, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, y, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }
        }
        return false
    }

    private fun findPathN(
        baseX: Int,
        baseY: Int,
        z: Int,
        localDestX: Int,
        localDestY: Int,
        destWidth: Int,
        destHeight: Int,
        srcSize: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
        reachStrategy: ReachStrategy
    ): Boolean {
        var x: Int
        var y: Int
        var dirFlag: Int
        val relativeSearchSize = searchMapSize - srcSize
        while (bufWriterIndex != bufReaderIndex) {
            currLocalX = validLocalX[bufReaderIndex]
            currLocalY = validLocalY[bufReaderIndex]
            bufReaderIndex = (bufReaderIndex + 1) and (ringBufferSize - 1)

            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    currLocalX + baseX,
                    currLocalY + baseY,
                    z,
                    localDestX + baseX,
                    localDestY + baseY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask,
                )
            ) {
                return true
            }

            val nextDistance = getDistance(currLocalX, currLocalY) + 1

            /* east to west */
            x = currLocalX - 1
            y = currLocalY
            dirFlag = DirectionFlag.EAST
            if (currLocalX > 0 && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_WEST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + srcSize - 1, z],
                        CollisionFlag.BLOCK_NORTH_WEST or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlagToCheck
                val blocked = (1 until srcSize - 1).any {
                    !collision.canMove(
                        flags[baseX, baseY, x, currLocalY + it, z],
                        clipFlag
                    )
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* west to east */
            x = currLocalX + 1
            y = currLocalY
            dirFlag = DirectionFlag.WEST
            if (currLocalX < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, currLocalY + srcSize - 1, z],
                        CollisionFlag.BLOCK_NORTH_EAST or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlagToCheck
                val blocked = (1 until srcSize - 1).any {
                    !collision.canMove(flags[baseX, baseY, currLocalX + srcSize, currLocalY + it, z], clipFlag)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* north to south  */
            x = currLocalX
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH
            if (currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_WEST or extraFlagToCheck) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize - 1, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlagToCheck
                val blocked = (1 until srcSize - 1).any {
                    !collision.canMove(
                        flags[baseX, baseY, currLocalX + it, y, z],
                        clipFlag
                    )
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* south to north */
            x = currLocalX
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH
            if (currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_WEST or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize - 1, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_EAST or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlagToCheck
                val blocked =
                    (1 until srcSize - 1).any {
                        !collision.canMove(
                            flags[baseX, baseY, x + it, currLocalY + srcSize, z],
                            clipFlag
                        )
                    }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* north-east to south-west */
            x = currLocalX - 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_EAST
            if (currLocalX > 0 && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], CollisionFlag.BLOCK_SOUTH_WEST or extraFlagToCheck)
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, x, currLocalY + it - 1, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + it - 1, y, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* north-west to south-east */
            x = currLocalX + 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST or extraFlagToCheck
                    )
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_NORTH_EAST_AND_WEST or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, currLocalX + srcSize, currLocalY + it - 1, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + it, y, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* south-east to north-west */
            x = currLocalX - 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_EAST
            if (currLocalX > 0 && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_WEST or extraFlagToCheck
                    )
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, x, currLocalY + it, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + it - 1, currLocalY + srcSize, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* south-west to north-east */
            x = currLocalX + 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_EAST or extraFlagToCheck
                    )
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, currLocalX + it, currLocalY + srcSize, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + srcSize, currLocalY + it, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }
        }
        return false
    }

    private fun findPath1RouteBlocker(
        baseX: Int,
        baseY: Int,
        z: Int,
        localDestX: Int,
        localDestY: Int,
        destWidth: Int,
        destHeight: Int,
        srcSize: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
        reachStrategy: ReachStrategy
    ): Boolean {
        var x: Int
        var y: Int
        var clipFlag: Int
        var dirFlag: Int
        val relativeSearchSize = searchMapSize - 1
        while (bufWriterIndex != bufReaderIndex) {
            currLocalX = validLocalX[bufReaderIndex]
            currLocalY = validLocalY[bufReaderIndex]
            bufReaderIndex = (bufReaderIndex + 1) and (ringBufferSize - 1)

            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    currLocalX + baseX,
                    currLocalY + baseY,
                    z,
                    localDestX + baseX,
                    localDestY + baseY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask,
                )
            ) {
                return true
            }

            val nextDistance = getDistance(currLocalX, currLocalY) + 1

            /* east to west */
            x = currLocalX - 1
            y = currLocalY
            clipFlag = CollisionFlag.BLOCK_WEST_ROUTE_BLOCKER or extraFlagToCheck
            dirFlag = DirectionFlag.EAST
            if (currLocalX > 0 && getDirection(x, y) == 0 && collision.canMove(
                    flags[baseX, baseY, x, y, z],
                    clipFlag
                )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* west to east */
            x = currLocalX + 1
            y = currLocalY
            clipFlag = CollisionFlag.BLOCK_EAST_ROUTE_BLOCKER or extraFlagToCheck
            dirFlag = DirectionFlag.WEST
            if (currLocalX < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], clipFlag)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north to south  */
            x = currLocalX
            y = currLocalY - 1
            clipFlag = CollisionFlag.BLOCK_SOUTH_ROUTE_BLOCKER or extraFlagToCheck
            dirFlag = DirectionFlag.NORTH
            if (currLocalY > 0 && getDirection(x, y) == 0 && collision.canMove(
                    flags[baseX, baseY, x, y, z],
                    clipFlag
                )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south to north */
            x = currLocalX
            y = currLocalY + 1
            clipFlag = CollisionFlag.BLOCK_NORTH_ROUTE_BLOCKER or extraFlagToCheck
            dirFlag = DirectionFlag.SOUTH
            if (currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(flags[baseX, baseY, x, y, z], clipFlag)
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-east to south-west */
            x = currLocalX - 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_EAST
            if (currLocalX > 0 && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, y, z],
                        CollisionFlag.BLOCK_SOUTH_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-west to south-east */
            x = currLocalX + 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, y, z],
                        CollisionFlag.BLOCK_SOUTH_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-east to north-west */
            x = currLocalX - 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_EAST
            if (currLocalX > 0 && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_NORTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, y, z],
                        CollisionFlag.BLOCK_NORTH_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-west to north-east */
            x = currLocalX + 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_NORTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, y, z],
                        CollisionFlag.BLOCK_NORTH_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }
        }
        return false
    }

    private fun findPath2RouteBlocker(
        baseX: Int,
        baseY: Int,
        z: Int,
        localDestX: Int,
        localDestY: Int,
        destWidth: Int,
        destHeight: Int,
        srcSize: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
        reachStrategy: ReachStrategy
    ): Boolean {
        var x: Int
        var y: Int
        var dirFlag: Int
        val relativeSearchSize = searchMapSize - 2
        while (bufWriterIndex != bufReaderIndex) {
            currLocalX = validLocalX[bufReaderIndex]
            currLocalY = validLocalY[bufReaderIndex]
            bufReaderIndex = (bufReaderIndex + 1) and (ringBufferSize - 1)

            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    currLocalX + baseX,
                    currLocalY + baseY,
                    z,
                    localDestX + baseX,
                    localDestY + baseY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask,
                )
            ) {
                return true
            }

            val nextDistance = getDistance(currLocalX, currLocalY) + 1

            /* east to west */
            x = currLocalX - 1
            y = currLocalY
            dirFlag = DirectionFlag.EAST
            if (currLocalX > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 1, z],
                        CollisionFlag.BLOCK_NORTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* west to east */
            x = currLocalX + 1
            y = currLocalY
            dirFlag = DirectionFlag.WEST
            if (currLocalX < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, currLocalY + 1, z],
                        CollisionFlag.BLOCK_NORTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north to south  */
            x = currLocalX
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH
            if (currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 1, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south to north */
            x = currLocalX
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH
            if (currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 1, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-east to south-west */
            x = currLocalX - 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_EAST
            if (currLocalX > 0 && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, y, z],
                        CollisionFlag.BLOCK_NORTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* north-west to south-east */
            x = currLocalX + 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_NORTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, currLocalY, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-east to north-west */
            x = currLocalX - 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_EAST
            if (currLocalX > 0 && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX, currLocalY + 2, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }

            /* south-west to north-east */
            x = currLocalX + 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + 2, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, currLocalY + 2, z],
                        CollisionFlag.BLOCK_NORTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + 2, y, z],
                        CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                setNextValidLocalCoords(x, y, dirFlag, nextDistance)
            }
        }
        return false
    }

    private fun findPathNRouteBlocker(
        baseX: Int,
        baseY: Int,
        z: Int,
        localDestX: Int,
        localDestY: Int,
        destWidth: Int,
        destHeight: Int,
        srcSize: Int,
        objRot: Int,
        objShape: Int,
        accessBitMask: Int,
        extraFlagToCheck: Int,
        collision: CollisionStrategy,
        reachStrategy: ReachStrategy
    ): Boolean {
        var x: Int
        var y: Int
        var dirFlag: Int
        val relativeSearchSize = searchMapSize - srcSize
        while (bufWriterIndex != bufReaderIndex) {
            currLocalX = validLocalX[bufReaderIndex]
            currLocalY = validLocalY[bufReaderIndex]
            bufReaderIndex = (bufReaderIndex + 1) and (ringBufferSize - 1)

            if (reachStrategy.reached(
                    flags,
                    defaultFlag,
                    currLocalX + baseX,
                    currLocalY + baseY,
                    z,
                    localDestX + baseX,
                    localDestY + baseY,
                    destWidth,
                    destHeight,
                    srcSize,
                    objRot,
                    objShape,
                    accessBitMask,
                )
            ) {
                return true
            }

            val nextDistance = getDistance(currLocalX, currLocalY) + 1

            /* east to west */
            x = currLocalX - 1
            y = currLocalY
            dirFlag = DirectionFlag.EAST
            if (currLocalX > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + srcSize - 1, z],
                        CollisionFlag.BLOCK_NORTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked = (1 until srcSize - 1).any {
                    !collision.canMove(
                        flags[baseX, baseY, x, currLocalY + it, z],
                        clipFlag
                    )
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* west to east */
            x = currLocalX + 1
            y = currLocalY
            dirFlag = DirectionFlag.WEST
            if (currLocalX < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, currLocalY + srcSize - 1, z],
                        CollisionFlag.BLOCK_NORTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked = (1 until srcSize - 1).any {
                    !collision.canMove(flags[baseX, baseY, currLocalX + srcSize, currLocalY + it, z], clipFlag)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* north to south  */
            x = currLocalX
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH
            if (currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize - 1, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_NORTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked = (1 until srcSize - 1).any {
                    !collision.canMove(
                        flags[baseX, baseY, currLocalX + it, y, z],
                        clipFlag
                    )
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* south to north */
            x = currLocalX
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH
            if (currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    ) &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize - 1, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag = CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked =
                    (1 until srcSize - 1).any {
                        !collision.canMove(
                            flags[baseX, baseY, x + it, currLocalY + srcSize, z],
                            clipFlag
                        )
                    }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* north-east to south-west */
            x = currLocalX - 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_EAST
            if (currLocalX > 0 && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, y, z],
                        CollisionFlag.BLOCK_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_NORTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, x, currLocalY + it - 1, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + it - 1, y, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* north-west to south-east */
            x = currLocalX + 1
            y = currLocalY - 1
            dirFlag = DirectionFlag.NORTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY > 0 && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, y, z],
                        CollisionFlag.BLOCK_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_NORTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, currLocalX + srcSize, currLocalY + it - 1, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + it, y, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* south-east to north-west */
            x = currLocalX - 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_EAST
            if (currLocalX > 0 && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, x, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, x, currLocalY + it, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + it - 1, currLocalY + srcSize, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }

            /* south-west to north-east */
            x = currLocalX + 1
            y = currLocalY + 1
            dirFlag = DirectionFlag.SOUTH_WEST
            if (currLocalX < relativeSearchSize && currLocalY < relativeSearchSize && getDirection(x, y) == 0 &&
                collision.canMove(
                        flags[baseX, baseY, currLocalX + srcSize, currLocalY + srcSize, z],
                        CollisionFlag.BLOCK_NORTH_EAST_ROUTE_BLOCKER or extraFlagToCheck
                    )
            ) {
                val clipFlag1 = CollisionFlag.BLOCK_SOUTH_EAST_AND_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val clipFlag2 = CollisionFlag.BLOCK_NORTH_AND_SOUTH_WEST_ROUTE_BLOCKER or extraFlagToCheck
                val blocked = (1 until srcSize).any {
                    !collision.canMove(flags[baseX, baseY, currLocalX + it, currLocalY + srcSize, z], clipFlag1) ||
                        !collision.canMove(flags[baseX, baseY, currLocalX + srcSize, currLocalY + it, z], clipFlag2)
                }
                if (!blocked) {
                    setNextValidLocalCoords(x, y, dirFlag, nextDistance)
                }
            }
        }
        return false
    }

    private fun findClosestApproachPoint(
        localSrcX: Int,
        localSrcY: Int,
        localDestX: Int,
        localDestY: Int,
        width: Int,
        length: Int,
    ): Boolean {
        var lowestCost = MAX_ALTERNATIVE_ROUTE_LOWEST_COST
        var maxAlternativePath = MAX_ALTERNATIVE_ROUTE_SEEK_RANGE
        val alternativeRouteRange = MAX_ALTERNATIVE_ROUTE_DISTANCE_FROM_DESTINATION
        val radiusX = localDestX - alternativeRouteRange..localDestX + alternativeRouteRange
        val radiusY = localDestY - alternativeRouteRange..localDestY + alternativeRouteRange
        for (x in radiusX) {
            for (y in radiusY) {
                if (x !in 0 until searchMapSize ||
                    y !in 0 until searchMapSize ||
                    getDistance(x, y) >= MAX_ALTERNATIVE_ROUTE_SEEK_RANGE
                ) {
                    continue
                }

                val dx = if (x < localDestX) {
                    localDestX - x
                } else if (x > localDestX + width - 1) {
                    x - (width + localDestX - 1)
                } else {
                    0
                }

                val dy = if (y < localDestY) {
                    localDestY - y
                } else if (y > localDestY + length - 1) {
                    y - (localDestY + length - 1)
                } else {
                    0
                }
                val cost = dx * dx + dy * dy
                if (cost < lowestCost || (cost == lowestCost && maxAlternativePath > getDistance(x, y))) {
                    currLocalX = x
                    currLocalY = y
                    lowestCost = cost
                    maxAlternativePath = getDistance(x, y)
                }
            }
        }
        return !(
            lowestCost == MAX_ALTERNATIVE_ROUTE_LOWEST_COST ||
                localSrcX == currLocalX && localSrcY == currLocalY
            )
    }

    private fun reset() {
        graphInfo.fill(DEFAULT_DISTANCE_VALUE shl 7)
        bufReaderIndex = 0
        bufWriterIndex = 0
    }

    private fun setNextValidLocalCoords(localX: Int, localY: Int, direction: Int, distance: Int) {
        val pathIndex = (localY * searchMapSize) + localX
        val bitpacked = direction or (distance shl 7)
        graphInfo[pathIndex] = bitpacked
        validLocalX[bufWriterIndex] = localX
        validLocalY[bufWriterIndex] = localY
        bufWriterIndex = (bufWriterIndex + 1) and (ringBufferSize - 1)
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline fun getDistance(localX: Int, localY: Int): Int {
        val pathIndex = (localY * searchMapSize) + localX
        return graphInfo[pathIndex] ushr 7
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline fun getDirection(localX: Int, localY: Int): Int {
        val pathIndex = (localY * searchMapSize) + localX
        return graphInfo[pathIndex] and 0x7F
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline operator fun IntArray.get(x: Int, y: Int): Int {
        val index = (y * searchMapSize) + x
        return this[index]
    }

    @Suppress("NOTHING_TO_INLINE")
    private inline operator fun Array<IntArray?>.get(baseX: Int, baseY: Int, localX: Int, localY: Int, z: Int): Int {
        val x = baseX + localX
        val y = baseY + localY
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

    private companion object {
        private val FAILED_ROUTE = Route(ArrayDeque(), alternative = false, success = false)
    }
}
