using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Pathfinding
{
    public class Polygon
    {
        public readonly List<Vertex> Vertices;
        public readonly Edge[] Edges; 

        public Polygon(Vector3[] vertices)
        {
            Vertices = new List<Vertex>();
            Edges = new Edge[vertices.Length];

            foreach (var v in vertices)
            {
                Vertices.Add(new Vertex(v));
            }

            for (var i = 1; i < Vertices.Count; i++)
            {
                var e = new Edge(Vertices[i - 1], Vertices[i]);
                Vertices[i - 1].Edge1 = e;
                Vertices[i].Edge2 = e;
                Edges[i - 1] = e;
            }

            var eLast = new Edge(Vertices.Last(), Vertices[0]);
            Vertices.Last().Edge1 = eLast;
            Vertices[0].Edge2 = eLast;
            Edges[vertices.Length - 1] = eLast;
        }

        public void Move(Vector3 moveVec)
        {
            foreach (var vertex in Vertices)
            {
                vertex.Move(moveVec);
            }
        }

        public bool IntersectsWith(float v1X, float v1Z, float v2X, float v2Z)
        {
            for (int i = 0; i < Edges.Length; i++)
            {
                if (Edges[i].IntersectsWith(v1X, v1Z, v2X, v2Z))
                {
                    return true;
                }
            }

            return false;
        }
    }
}
