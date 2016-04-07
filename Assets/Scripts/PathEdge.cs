using System;
using UnityEngine;

namespace Pathfinding
{
    public class PathEdge : IEquatable<PathEdge>
    {
        public Vertex Vertex1 { get; private set; }
        public Vertex Vertex2 { get; private set; }

        public float Weight { get; private set; }

        public PathEdge(Vertex v1, Vertex v2)
        {
            Vertex1 = v1;
            Vertex2 = v2;

            Weight = Vector3.Distance(v1.Position, v2.Position);
        }

        public bool Equals(PathEdge other)
        {
            return (Vertex1 == other.Vertex1 && Vertex2 == other.Vertex2)
                   || (Vertex1 == other.Vertex2 && Vertex2 == other.Vertex1);
        }
    }
}
