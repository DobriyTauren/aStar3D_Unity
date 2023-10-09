using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Unity.VisualScripting;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

public class BlockSpawnController : MonoBehaviour
{
    public GameObject spawnEntity;
    public GameObject spawnPathEntity;

    public int endX = 5;
    public int endY = 5;
    public int endZ = 5;

    public int MatrixSize = 7;

    private Node3D start = new Node3D(0, 0, 0);
    private Node3D end;

    private bool _isPathCreated = false;

    private int[,,] _grid;
    private GameObject[,,] objectGrid;

    private DateTime _startTime;

    public class Node3D
    {
        public int X { get; set; }
        public int Y { get; set; }
        public int Z { get; set; }
        public int G { get; set; } // Стоимость движения от начальной точки
        public int H { get; set; } // Эвристическая оценка до конечной точки
        public int F => G + H;     // Сумма G и H
        public Node3D Parent { get; set; } // Родительская точка для восстановления пути

        public Node3D(int x, int y, int z)
        {
            X = x;
            Y = y;
            Z = z;
        }
    }

    public class AStar3D_BinaryTree
    {
        public static List<Node3D> FindPath(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            // Инициализируем бинарное дерево
            BinarySearchTree openList = new BinarySearchTree();
            openList.Insert(start);

            List<Node3D> closedList = new List<Node3D>();
            object lockObject = new object(); // Объект для блокировки доступа к общим данным

            while (openList.Count > 0)
            {
                Node3D current = openList.ExtractMin();

                lock (lockObject)
                {
                    closedList.Add(current);
                }

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей
                int[] dx = { -1, 1, 0, 0, 0, 0 };
                int[] dy = { 0, 0, -1, 1, 0, 0 };
                int[] dz = { 0, 0, 0, 0, -1, 1 };

                Parallel.For(0, 6, i =>
                {
                    int newX = current.X + dx[i];
                    int newY = current.Y + dy[i];
                    int newZ = current.Z + dz[i];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);

                        bool inClosedList;
                        lock (lockObject)
                        {
                            inClosedList = closedList.Any(c => c.X == neighbor.X && c.Y == neighbor.Y && c.Z == neighbor.Z);
                        }

                        if (!inClosedList)
                        {
                            int tentativeG = current.G + 1;

                            bool inOpenList;
                            lock (lockObject)
                            {
                                inOpenList = openList.Contains(neighbor);
                            }

                            if (!inOpenList || tentativeG < neighbor.G)
                            {
                                neighbor.Parent = current;
                                neighbor.G = tentativeG;
                                neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                                lock (lockObject)
                                {
                                    if (!inOpenList)
                                    {
                                        openList.Insert(neighbor);
                                    }
                                }
                            }
                        }
                    }
                });
            }

            // Путь не найден
            return null;
        }

        private static List<Node3D> ReconstructPath(Node3D node)
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

        private static bool IsInsideGrid(int x, int y, int z, int width, int height, int depth)
        {
            return x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth;
        }
    }

    public class BinarySearchTree
    {
        private Node3DNode root;

        public bool Contains(Node3D node)
        {
            return Search(root, node) != null;
        }

        public void Insert(Node3D node)
        {
            root = InsertRec(root, node);
        }

        public Node3D ExtractMin()
        {
            if (root == null)
            {
                throw new InvalidOperationException("The tree is empty.");
            }

            Node3D minNode = FindMin(root);
            root = DeleteRec(root, minNode);
            return minNode;
        }

        public int Count
        {
            get { return CountNodes(root); }
        }

        private Node3DNode Search(Node3DNode root, Node3D node)
        {
            if (root == null || (root.Value.X == node.X && root.Value.Y == node.Y && root.Value.Z == node.Z))
            {
                return root;
            }

            if (root.Value.F > node.F)
            {
                return Search(root.Left, node);
            }
            else
            {
                return Search(root.Right, node);
            }
        }

        private Node3DNode InsertRec(Node3DNode root, Node3D node)
        {
            if (root == null)
            {
                return new Node3DNode(node);
            }

            if (node.F < root.Value.F)
            {
                root.Left = InsertRec(root.Left, node);
            }
            else
            {
                root.Right = InsertRec(root.Right, node);
            }

            return root;
        }

        private Node3D FindMin(Node3DNode root)
        {
            while (root.Left != null)
            {
                root = root.Left;
            }

            return root.Value;
        }

        private Node3DNode DeleteRec(Node3DNode root, Node3D node)
        {
            if (root == null)
            {
                return root;
            }

            if (node.F < root.Value.F)
            {
                root.Left = DeleteRec(root.Left, node);
            }
            else if (node.F > root.Value.F)
            {
                root.Right = DeleteRec(root.Right, node);
            }
            else
            {
                if (root.Left == null)
                {
                    return root.Right;
                }
                else if (root.Right == null)
                {
                    return root.Left;
                }

                root.Value = FindMin(root.Right);
                root.Right = DeleteRec(root.Right, root.Value);
            }

            return root;
        }

        private int CountNodes(Node3DNode root)
        {
            if (root == null)
            {
                return 0;
            }

            return 1 + CountNodes(root.Left) + CountNodes(root.Right);
        }
    }

    public class Node3DNode
    {
        public Node3D Value { get; set; }
        public Node3DNode Left { get; set; }
        public Node3DNode Right { get; set; }

        public Node3DNode(Node3D value)
        {
            Value = value;
        }
    }

    public class AStar3D
    {
        public static List<Node3D> FindPathOneThread(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            List<Node3D> openList = new List<Node3D>();
            List<Node3D> closedList = new List<Node3D>();

            openList.Add(start);

            while (openList.Count > 0)
            {
                Node3D current = openList[0];

                // Находим узел с наименьшей оценкой F
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].F < current.F)
                    {
                        current = openList[i];
                    }
                }

                openList.Remove(current);
                closedList.Add(current);

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей
                int[] dx = { -1, 1, 0, 0, 0, 0 };
                int[] dy = { 0, 0, -1, 1, 0, 0 };
                int[] dz = { 0, 0, 0, 0, -1, 1 };

                for (int i = 0; i < 6; i++)
                {
                    int newX = current.X + dx[i];
                    int newY = current.Y + dy[i];
                    int newZ = current.Z + dz[i];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);

                        if (closedList.Contains(neighbor))
                        {
                            continue;
                        }

                        int tentativeG = current.G + 1;

                        if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                        {
                            neighbor.Parent = current;
                            neighbor.G = tentativeG;
                            neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                            if (!openList.Contains(neighbor))
                            {
                                openList.Add(neighbor);
                            }
                        }
                    }
                }
            }

            // Путь не найден
            return null;
        }

        public static List<Node3D> FindPathMultiThread_ParallelFor(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            List<Node3D> openList = new List<Node3D>();
            List<Node3D> closedList = new List<Node3D>();
            object lockObject = new object(); // Объект для блокировки доступа к общим данным

            openList.Add(start);

            while (openList.Count > 0)
            {
                Node3D current = openList[0];

                // Находим узел с наименьшей оценкой F
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].F < current.F)
                    {
                        current = openList[i];
                    }
                }

                openList.Remove(current);
                closedList.Add(current);

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей
                int[] dx = { -1, 1, 0, 0, 0, 0 };
                int[] dy = { 0, 0, -1, 1, 0, 0 };
                int[] dz = { 0, 0, 0, 0, -1, 1 };

                // Используем параллельную обработку соседей
                Parallel.For(0, 6, i =>
                {
                    int newX = current.X + dx[i];
                    int newY = current.Y + dy[i];
                    int newZ = current.Z + dz[i];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);

                        lock (lockObject) // Блокируем доступ к общим данным
                        {
                            if (closedList.Contains(neighbor))
                            {
                                return;
                            }

                            int tentativeG = current.G + 1;

                            if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                            {
                                neighbor.Parent = current;
                                neighbor.G = tentativeG;
                                neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                                if (!openList.Contains(neighbor))
                                {
                                    openList.Add(neighbor);
                                }
                            }
                        }
                    }
                });
            }

            // Путь не найден
            return null;
        }

        public static List<Node3D> FindPathMultiThread_ParallelForEach(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            List<Node3D> openList = new List<Node3D>();
            List<Node3D> closedList = new List<Node3D>();
            object lockObject = new object(); // Объект для блокировки доступа к общим данным

            openList.Add(start);

            while (openList.Count > 0)
            {
                Node3D current = openList[0];

                // Находим узел с наименьшей оценкой F
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].F < current.F)
                    {
                        current = openList[i];
                    }
                }

                openList.Remove(current);
                closedList.Add(current);

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей с использованием параллельной обработки
                Parallel.ForEach(GetNeighborDirections(), direction =>
                {
                    int newX = current.X + direction[0];
                    int newY = current.Y + direction[1];
                    int newZ = current.Z + direction[2];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);

                        lock (lockObject) // Блокируем доступ к общим данным
                        {
                            if (closedList.Contains(neighbor))
                            {
                                return;
                            }

                            int tentativeG = current.G + 1;

                            if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                            {
                                neighbor.Parent = current;
                                neighbor.G = tentativeG;
                                neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                                if (!openList.Contains(neighbor))
                                {
                                    openList.Add(neighbor);
                                }
                            }
                        }
                    }
                });
            }

            // Путь не найден
            return null;
        }

        public static List<Node3D> FindPath_PLINQ(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            List<Node3D> openList = new List<Node3D>();
            List<Node3D> closedList = new List<Node3D>();
            object lockObject = new object(); // Объект для блокировки доступа к общим данным

            openList.Add(start);

            while (openList.Count > 0)
            {
                Node3D current = openList[0];

                // Находим узел с наименьшей оценкой F
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].F < current.F)
                    {
                        current = openList[i];
                    }
                }

                openList.Remove(current);
                closedList.Add(current);

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей с использованием PLINQ
                var neighborDirections = GetNeighborDirections();
                var parallelQuery = neighborDirections.AsParallel().WithDegreeOfParallelism(Environment.ProcessorCount);

                parallelQuery.ForAll(direction =>
                {
                    int newX = current.X + direction[0];
                    int newY = current.Y + direction[1];
                    int newZ = current.Z + direction[2];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);

                        lock (lockObject) // Блокируем доступ к общим данным
                        {
                            if (closedList.Contains(neighbor))
                            {
                                return;
                            }

                            int tentativeG = current.G + 1;

                            if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                            {
                                neighbor.Parent = current;
                                neighbor.G = tentativeG;
                                neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                                if (!openList.Contains(neighbor))
                                {
                                    openList.Add(neighbor);
                                }
                            }
                        }
                    }
                });
            }

            // Путь не найден
            return null;
        }

        public static List<Node3D> FindPath_TPL(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            List<Node3D> openList = new List<Node3D>();
            List<Node3D> closedList = new List<Node3D>();
            object lockObject = new object(); // Объект для блокировки доступа к общим данным

            openList.Add(start);

            while (openList.Count > 0)
            {
                Node3D current = openList[0];

                // Находим узел с наименьшей оценкой F
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].F < current.F)
                    {
                        current = openList[i];
                    }
                }

                openList.Remove(current);
                closedList.Add(current);

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей с использованием асинхронных задач
                var neighborDirections = GetNeighborDirections();

                // Используем ConcurrentBag для сбора результатов асинхронных задач
                ConcurrentBag<Node3D> newNodes = new ConcurrentBag<Node3D>();

                Parallel.ForEach(neighborDirections, direction =>
                {
                    int newX = current.X + direction[0];
                    int newY = current.Y + direction[1];
                    int newZ = current.Z + direction[2];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);

                        lock (lockObject) // Блокируем доступ к общим данным
                        {
                            if (closedList.Contains(neighbor))
                            {
                                return;
                            }

                            int tentativeG = current.G + 1;

                            if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                            {
                                neighbor.Parent = current;
                                neighbor.G = tentativeG;
                                neighbor.H = Math.Abs(newX - end.X) + Math.Abs(newY - end.Y) + Math.Abs(newZ - end.Z);

                                if (!openList.Contains(neighbor))
                                {
                                    newNodes.Add(neighbor);
                                }
                            }
                        }
                    }
                });

                // Добавляем новые узлы в openList
                foreach (var newNode in newNodes)
                {
                    openList.Add(newNode);
                }
            }

            // Путь не найден
            return null;
        }

        public static List<Node3D> FindPath_ThreadPool(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            List<Node3D> openList = new List<Node3D>();
            List<Node3D> closedList = new List<Node3D>();
            object lockObject = new object(); // Объект для блокировки доступа к общим данным

            openList.Add(start);

            while (openList.Count > 0)
            {
                Node3D current = openList[0];

                // Находим узел с наименьшей оценкой F
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].F < current.F)
                    {
                        current = openList[i];
                    }
                }

                openList.Remove(current);
                closedList.Add(current);

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей
                int[] dx = { -1, 1, 0, 0, 0, 0 };
                int[] dy = { 0, 0, -1, 1, 0, 0 };
                int[] dz = { 0, 0, 0, 0, -1, 1 };

                List<Node3D> neighbors = new List<Node3D>();

                for (int i = 0; i < 6; i++)
                {
                    int newX = current.X + dx[i];
                    int newY = current.Y + dy[i];
                    int newZ = current.Z + dz[i];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);
                        neighbors.Add(neighbor);
                    }
                }

                // Параллельно обрабатываем соседей
                Parallel.ForEach(neighbors, neighbor =>
                {
                    lock (lockObject)
                    {
                        if (closedList.Contains(neighbor))
                        {
                            return;
                        }

                        int tentativeG = current.G + 1;

                        if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                        {
                            neighbor.Parent = current;
                            neighbor.G = tentativeG;
                            neighbor.H = Math.Abs(neighbor.X - end.X) + Math.Abs(neighbor.Y - end.Y) + Math.Abs(neighbor.Z - end.Z);

                            if (!openList.Contains(neighbor))
                            {
                                openList.Add(neighbor);
                            }
                        }
                    }
                });
            }

            // Путь не найден
            return null;
        }

        public static List<Node3D> FindPath_AsyncTasks(int[,,] grid, Node3D start, Node3D end)
        {
            int width = grid.GetLength(0);
            int height = grid.GetLength(1);
            int depth = grid.GetLength(2);

            List<Node3D> openList = new List<Node3D>();
            List<Node3D> closedList = new List<Node3D>();
            object lockObject = new object(); // Объект для блокировки доступа к общим данным

            openList.Add(start);

            while (openList.Count > 0)
            {
                Node3D current = openList[0];

                // Находим узел с наименьшей оценкой F
                for (int i = 1; i < openList.Count; i++)
                {
                    if (openList[i].F < current.F)
                    {
                        current = openList[i];
                    }
                }

                openList.Remove(current);
                closedList.Add(current);

                if (current.X == end.X && current.Y == end.Y && current.Z == end.Z)
                {
                    // Путь найден, восстанавливаем его
                    return ReconstructPath(current);
                }

                // Генерация соседей
                int[] dx = { -1, 1, 0, 0, 0, 0 };
                int[] dy = { 0, 0, -1, 1, 0, 0 };
                int[] dz = { 0, 0, 0, 0, -1, 1 };

                List<Task> neighborTasks = new List<Task>();

                for (int i = 0; i < 6; i++)
                {
                    int newX = current.X + dx[i];
                    int newY = current.Y + dy[i];
                    int newZ = current.Z + dz[i];

                    if (IsInsideGrid(newX, newY, newZ, width, height, depth) && grid[newX, newY, newZ] == 0)
                    {
                        Node3D neighbor = new Node3D(newX, newY, newZ);
                        neighborTasks.Add(ProcessNeighborAsync(neighbor, current, end, openList, closedList, lockObject));
                    }
                }

                Task.WhenAll(neighborTasks).Wait(); // Ждем завершения всех задач соседей
            }

            // Путь не найден
            return null;
        }


        private static async Task ProcessNeighborAsync(Node3D neighbor, Node3D current, Node3D end, List<Node3D> openList, List<Node3D> closedList, object lockObject)
        {
            lock (lockObject)
            {
                if (closedList.Contains(neighbor))
                {
                    return;
                }

                int tentativeG = current.G + 1;

                if (!openList.Contains(neighbor) || tentativeG < neighbor.G)
                {
                    neighbor.Parent = current;
                    neighbor.G = tentativeG;
                    neighbor.H = Math.Abs(neighbor.X - end.X) + Math.Abs(neighbor.Y - end.Y) + Math.Abs(neighbor.Z - end.Z);

                    if (!openList.Contains(neighbor))
                    {
                        openList.Add(neighbor);
                    }
                }
            }
        }

        private static List<int[]> GetNeighborDirections()
        {
            // Возвращает массив направлений движения [dx, dy, dz]
            return new List<int[]>
    {
        new int[] { -1, 0, 0 },
        new int[] { 1, 0, 0 },
        new int[] { 0, -1, 0 },
        new int[] { 0, 1, 0 },
        new int[] { 0, 0, -1 },
        new int[] { 0, 0, 1 }
    };
        }

        private static List<Node3D> ReconstructPath(Node3D node)
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

        private static bool IsInsideGrid(int x, int y, int z, int width, int height, int depth)
        {
            return x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < depth;
        }
    }

    void Start()
    {
       
    }

    private void PathfindingStart()
    {


        FindPathAlgorithm[] findPathAlgorithms = new FindPathAlgorithm[]
        {
            AStar3D_BinaryTree.FindPath,
            //AStar3D.FindPathOneThread,
            //AStar3D.FindPath_PLINQ, 
            //AStar3D.FindPathMultiThread_ParallelFor, AStar3D.FindPathMultiThread_ParallelForEach,
            //AStar3D.FindPath_TPL, AStar3D.FindPath_ThreadPool, AStar3D.FindPath_AsyncTasks,
        };

        _startTime = DateTime.Now;

        _isPathCreated = CheckAlgorithmsSpeed(_grid, start, end, findPathAlgorithms);

        Debug.Log($"------------------------------------\ndone! time spend: {(DateTime.Now - _startTime).TotalSeconds}(s)\n");

    }

    private bool CheckAlgorithmsSpeed(int[,,] grid, Node3D start, Node3D end, params FindPathAlgorithm[] algorithms)
    {
        double prevSeconds = 0;
        List<Node3D> path = new List<Node3D>();

        for (int i = 0; i < algorithms.Length; i++)
        {
            path = algorithms[i](grid, start, end);

            if (path != null)
            {
                //foreach (Node3D node in path)
                //{
                //    Debug.Log($"({node.X}, {node.Y}, {node.Z})");
                //}

                double totalSeconds = (DateTime.Now - _startTime).TotalSeconds;
                Debug.Log($"[{i}] path found! time spend: {totalSeconds - prevSeconds}(s)");

                prevSeconds = totalSeconds;
                
                if (!_isPathCreated)
                {
                    CreatePath(path);
                }
            }
            else
            {
                Debug.Log($"[{i}] path not found.");

                return false;
            }
        }

        return true;
    }

    public void CreatePath(List<Node3D> path)
    {
        for (int i = 0; i < path.Count; i++) 
        {
            objectGrid[path[i].X, path[i].Y, path[i].Z] = Instantiate(spawnPathEntity, new Vector3(path[i].X, path[i].Y, path[i].Z), Quaternion.Euler(0, 0, 0));
        }
    }

    public void CreateMatrix(ref int[,,] grid)
    {
        grid = GenerateRandomGrid(MatrixSize, MatrixSize, MatrixSize);

        if (objectGrid != null)
        {
            ClearAllObjects();
        }

        objectGrid = new GameObject[MatrixSize, MatrixSize, MatrixSize];

        for (int i = 0; i < MatrixSize; i++)
        {
            for (int j = 0; j < MatrixSize; j++)
            {
                for (int k = 0; k < MatrixSize; k++)
                {
                    if (grid[i, j, k] == 1)
                    {
                        objectGrid[i,j,k] = Instantiate(spawnEntity, new Vector3(i, j, k), Quaternion.Euler(0, 0, 0));
                    }
                }
            }
        }
    }

    public void ClearAllObjects()
    {
        for (int i = 0; i < objectGrid.GetLength(0); i++)
        {
            for (int j = 0; j < objectGrid.GetLength(1); j++)
            {
                for (int k = 0; k < objectGrid.GetLength(2); k++)
                {
                    if (objectGrid[i, j, k] != null)
                    {
                        Destroy(objectGrid[i, j, k]);
                    }
                }
            }
        }
    }

    public delegate List<Node3D> FindPathAlgorithm(int[,,] grid, Node3D start, Node3D end);

    private int[,,] GenerateRandomGrid(int xSize, int ySize, int zSize)
    {
        System.Random random = new System.Random();
        int[,,] grid = new int[xSize, ySize, zSize];

        for (int x = 0; x < xSize; x++)
        {
            for (int y = 0; y < ySize; y++)
            {
                for (int z = 0; z < zSize; z++)
                {
                    // Генерируем случайное число от 0 до 1
                    int randomNumber = random.Next(2);
                    grid[x, y, z] = randomNumber;
                }
            }
        }

        grid[end.X, end.Y, end.Z] = 0;
        grid[start.X, start.Y, start.Z] = 0;

        return grid;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.R))
        {
            end = new Node3D(endX, endY, endZ);

            CreateMatrix(ref _grid);

            _isPathCreated = false;
        }
        else if (Input.GetKeyDown(KeyCode.F) && !_isPathCreated)
        {
            PathfindingStart();
        }
    }
}
