using System;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System.Linq;

namespace Pathfinding
{
    public class EdgeDistancePair : IComparable<EdgeDistancePair>
    {
        public Edge Edge { get; private set; }
        public float Distance { get; private set; }

        public EdgeDistancePair(Edge e, float t)
        {
            Edge = e;
            Distance = t;
        }

        public int CompareTo(EdgeDistancePair other)
        {
            if (other == null)
            {
                return 1;
            }
            return Distance.CompareTo(other.Distance);
        }
    }

    public class Edge : IComparable<Edge>
    {
        public float DistanceToReference { get; set; } 

        public Vertex Vertex1 { get; private set; }
        public Vertex Vertex2 { get; private set; }

        public Edge(Vertex vertex1, Vertex vertex2)
        {
            Vertex1 = vertex1;
            Vertex2 = vertex2;
        }

        public bool IntersectsWith(Vector3 other1, Vector3 other2)
        {
            // Stolen from: http://yunus.hacettepe.edu.tr/~burkay.genc/courses/bca608/slides/week3.pdf
            return (Util.Left(Vertex1.Position, Vertex2.Position, other1) ^ Util.Left(Vertex1.Position, Vertex2.Position, other2)) 
                && (Util.Left(other1, other2, Vertex1.Position) ^ Util.Left(other1, other2, Vertex2.Position));
        }

        public bool IntersectsWith(Edge e)
        {
            return IntersectsWith(e.Vertex1.Position, e.Vertex2.Position);
        }

        public bool IntersectsWith(Ray ray, out float t)
        {
            return Util.RayLineIntersection(ray, Vertex1.Position, Vertex2.Position, out t);
        }

        public int CompareTo(Edge other)
        {
            if (other == null)
            {
                return 1;
            }
            return DistanceToReference.CompareTo(other.DistanceToReference);

        }
    }

    public class Vertex
    {
        public Vector3 Position { get; private set; }
        public Edge Edge1 { get; set; }
        public Edge Edge2 { get; set; }

        public Vertex(Vector3 pos)
        {
            Position = pos;
        }

        public List<Edge> GetEdgesOnSide(bool isClockwise) // TODO: Could be cached?
        {
            var retVal = new List<Edge>();

            var e1Other = Edge1.Vertex1 == this ? Edge1.Vertex2 : Edge1.Vertex1;
            var e2Other = Edge2.Vertex1 == this ? Edge2.Vertex2 : Edge2.Vertex1;

            if (isClockwise)
            {
                if (e1Other.Position.z < Position.z)
                {
                    retVal.Add(Edge1);
                }

                if (e2Other.Position.z < Position.z)
                {
                    retVal.Add(Edge2);
                }

            }
            else
            {
                if (e1Other.Position.z > Position.z)
                {
                    retVal.Add(Edge1);
                }

                if (e2Other.Position.z > Position.z)
                {
                    retVal.Add(Edge2);
                }
            }


            return retVal;
        }

        
    }

    public class Polygon
    {
        public readonly List<Vertex> Vertices;
        public readonly List<Edge> Edges; 

        public Polygon(Vector3[] vertices)
        {
            Vertices = new List<Vertex>();
            Edges = new List<Edge>();

            foreach (var v in vertices)
            {
                Vertices.Add(new Vertex(v));
            }

            for (var i = 1; i < Vertices.Count; i++)
            {
                var e = new Edge(Vertices[i - 1], Vertices[i]);
                Vertices[i - 1].Edge1 = e;
                Vertices[i].Edge2 = e;
                Edges.Add(e);
            }

            var eLast = new Edge(Vertices.Last(), Vertices[0]);
            Vertices.Last().Edge1 = eLast;
            Vertices[0].Edge2 = eLast;
            Edges.Add(eLast);
        }

    }
}
