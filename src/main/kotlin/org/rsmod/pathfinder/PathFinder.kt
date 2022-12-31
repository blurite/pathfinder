package org.rsmod.pathfinder

import org.rsmod.pathfinder.collision.CollisionStrategies
import org.rsmod.pathfinder.collision.CollisionStrategy
import org.rsmod.pathfinder.reach.DefaultReachStrategy
import org.rsmod.pathfinder.reach.ReachStrategy

private const val DEFAULT_SRC_SIZE = 1
private const val DEFAULT_DEST_WIDTH = 0
private const val DEFAULT_DEST_HEIGHT = 0
private const val DEFAULT_MAX_TURNS = 25
private const val DEFAULT_OBJ_ROT = 10
private const val DEFAULT_OBJ_SHAPE = -1
private const val DEFAULT_ACCESS_BITMASK = 0

public interface PathFinder {

    public fun findPath(
        srcX: Int,
        srcY: Int,
        destX: Int,
        destY: Int,
        z: Int,
        srcSize: Int = DEFAULT_SRC_SIZE,
        destWidth: Int = DEFAULT_DEST_WIDTH,
        destHeight: Int = DEFAULT_DEST_HEIGHT,
        objRot: Int = DEFAULT_OBJ_ROT,
        objShape: Int = DEFAULT_OBJ_SHAPE,
        accessBitMask: Int = DEFAULT_ACCESS_BITMASK,
        maxTurns: Int = DEFAULT_MAX_TURNS,
        collision: CollisionStrategy = CollisionStrategies.Normal,
        reachStrategy: ReachStrategy = DefaultReachStrategy
    ): Route
}
