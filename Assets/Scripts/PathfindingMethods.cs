using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static BlockSpawnController;

public static class PathfindingMethods 
{
    public static bool IsLooping(int openListCount, int closedListCount, int limitNodes)
    {
        if (openListCount + closedListCount > limitNodes)
        {
            return true;
        }

        return false;
    }

    public static List<Node3D> ReconstructPath(Node3D node)
    {
        List<Node3D> path = new List<Node3D>();

        while (node != null)
        {
            path.Add(node);
            node = node.Parent;
        }

        path.Reverse();
        return path;
    }

    public static bool IsInsideGrid(int x, int y, int z, int width, int height, int depth)
    {
        return x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth;
    }

}
