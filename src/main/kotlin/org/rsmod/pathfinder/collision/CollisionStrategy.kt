package org.rsmod.pathfinder.collision

import org.rsmod.pathfinder.flag.CollisionFlag

public interface CollisionStrategy {
    public fun canMove(tileFlag: Int, blockFlag: Int): Boolean
}

public class NormalBlockFlagCollision : CollisionStrategy {
    override fun canMove(tileFlag: Int, blockFlag: Int): Boolean {
        return (tileFlag and blockFlag) == 0
    }
}

public class RoofBoundFlagCollision : CollisionStrategy {
    override fun canMove(tileFlag: Int, blockFlag: Int): Boolean {
        return (tileFlag and blockFlag) == 0 && (tileFlag and CollisionFlag.ROOF) != 0
    }
}

public class InverseBlockFlagCollision(
    private val inverseFlag: Int
) : CollisionStrategy {
    override fun canMove(tileFlag: Int, blockFlag: Int): Boolean {
        val flag = blockFlag and inverseFlag.inv()
        return (tileFlag and flag) == 0 && (tileFlag and inverseFlag) != 0
    }
}

public class LineOfSightBlockFlagCollision : CollisionStrategy {
    override fun canMove(tileFlag: Int, blockFlag: Int): Boolean {
        val movementFlags = (blockFlag and BLOCK_MOVEMENT) shl 9
        val routeFlags = (blockFlag and BLOCK_ROUTE) shr 13
        val finalBlockFlag = movementFlags or routeFlags
        return (tileFlag and finalBlockFlag) == 0
    }

    private companion object {
        private const val BLOCK_MOVEMENT = CollisionFlag.WALL_NORTH_WEST or
            CollisionFlag.WALL_NORTH or
            CollisionFlag.WALL_NORTH_EAST or
            CollisionFlag.WALL_EAST or
            CollisionFlag.WALL_SOUTH_EAST or
            CollisionFlag.WALL_SOUTH or
            CollisionFlag.WALL_SOUTH_WEST or
            CollisionFlag.WALL_WEST or
            CollisionFlag.OBJECT

        private const val BLOCK_ROUTE = CollisionFlag.WALL_NORTH_WEST_ROUTE_BLOCKER or
            CollisionFlag.WALL_NORTH_ROUTE_BLOCKER or
            CollisionFlag.WALL_NORTH_EAST_ROUTE_BLOCKER or
            CollisionFlag.WALL_EAST_ROUTE_BLOCKER or
            CollisionFlag.WALL_SOUTH_EAST_ROUTE_BLOCKER or
            CollisionFlag.WALL_SOUTH_ROUTE_BLOCKER or
            CollisionFlag.WALL_SOUTH_WEST_ROUTE_BLOCKER or
            CollisionFlag.WALL_WEST_ROUTE_BLOCKER or
            CollisionFlag.OBJECT_ROUTE_BLOCKER
    }
}
