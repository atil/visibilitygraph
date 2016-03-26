using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Pathfinding
{
    public struct Edge
    {
        public Vector3 A { get; private set; }
        public Vector3 B { get; private set; }

        public Edge(Vector3 a, Vector3 b) : this()
        {
            A = a;
            B = b;
        }
    }

    public class Polygon
    {
        public readonly List<Vector3> Vertices;
        public readonly List<Edge> Edges; 

        public Polygon(Vector3[] vertices)
        {
            Vertices = new List<Vector3>();
            Edges = new List<Edge>();

            foreach (var v in vertices)
            {
                Vertices.Add(v);
            }

            for (var i = 1; i < Vertices.Count; i++)
            {
                Edges.Add(new Edge(Vertices[i - 1], Vertices[i]));
            }
            Edges.Add(new Edge(Vertices.Last(), Vertices[0]));
        }

    }
}
