using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Pathfinding.SelfBalancedTree;
using UnityEngine;

namespace Pathfinding
{
    public class VisibilityGraph
    {
        private readonly AVLTree<Edge> _bst = new AVLTree<Edge>();
        private readonly List<Polygon> _polygons = new List<Polygon>();
        private readonly List<Vertex> _allVertices = new List<Vertex>();
        private readonly Dictionary<int, PathEdge> _pathEdgeDict = new Dictionary<int, PathEdge>();

        public void AddPolygon(Polygon polygon)
        {
            if (_polygons.Contains(polygon))
            {
                return;
            }

            _polygons.Add(polygon);
            foreach (var v in polygon.Vertices)
            {
                _allVertices.Add(v);
            }

            // Place polygon
            foreach (var vertex in polygon.Vertices)
            {
                CalculateVisiblityForVertex(vertex, _allVertices);
            }

            // Recalculate it's touching edges -- it might have obstructed someone's vision
            var touchingVertices = GetTouchingVertices(polygon);

            foreach (var touchingVertex in touchingVertices)
            {
                RemoveVertex(touchingVertex);
            }

            foreach (var touchingEdge in touchingVertices)
            {
                CalculateVisiblityForVertex(touchingEdge, _allVertices);
            }
        }

        private void RemoveVertex(Vertex vertex)
        {
            var edgeKeys = new HashSet<int>();

            foreach (var pathEdgeEntry in _pathEdgeDict)
            {
                if (pathEdgeEntry.Value.Vertex1 == vertex || pathEdgeEntry.Value.Vertex2 == vertex)
                {
                    edgeKeys.Add(pathEdgeEntry.Key);
                }
            }

            foreach (var edgeKey in edgeKeys)
            {
                _pathEdgeDict.Remove(edgeKey);
            }
        }

        public void RemovePolygon(Polygon polygon)
        {
            _polygons.Remove(polygon);
            foreach (var v in polygon.Vertices)
            {
                _allVertices.Remove(v);
            }
        }

        public void MovePolygon(Polygon polygon, Vector3 moveVec)
        {
            // TODO
        }

        private List<Vertex> GetTouchingVertices(Polygon polygon)
        {
            var retVal = new List<Vertex>();
            foreach (var pathEdge in _pathEdgeDict.Values)
            {
                if (polygon.Vertices.Contains(pathEdge.Vertex1)
                    && !polygon.Vertices.Contains(pathEdge.Vertex2))
                {
                    retVal.Add(pathEdge.Vertex2);
                }

                if (polygon.Vertices.Contains(pathEdge.Vertex2)
                    && !polygon.Vertices.Contains(pathEdge.Vertex1))
                {
                    retVal.Add(pathEdge.Vertex1);
                }
            }

            return retVal;
        }

        private void CalculateVisiblityForVertex(Vertex pivot, List<Vertex> allVertices)
        {
            var visibleVertices = GetVisibilePoints(pivot, allVertices);

            foreach (var visibleVertex in visibleVertices)
            {
                var key = pivot.GetHashCode() ^ visibleVertex.GetHashCode();
                if (!_pathEdgeDict.ContainsKey(key))
                {
                    _pathEdgeDict.Add(key, new PathEdge(pivot, visibleVertex));
                }
            }
        }

        private List<Vertex> GetVisibilePoints(Vertex v, List<Vertex> allVertices)
        {
            var visiblePoints = new List<Vertex>();
            var sortedEvents = Util.SortClockwise(v.Position, allVertices);

            var ray = new Ray(v.Position, Vector3.right);
            foreach (var polygon in _polygons)
            {
                foreach (var edge in polygon.Edges)
                {
                    float t;
                    if (edge.IntersectsWith(ray, out t))
                    {
                        edge.DistanceToReference = t;
                        _bst.Add(edge);
                    }
                }
            }

            foreach (var eventPoint in sortedEvents)
            {
                if (eventPoint == v)
                {
                    continue;
                }

                if (IsVisible(v, eventPoint))
                {
                    visiblePoints.Add(eventPoint);
                }

                // Algorithm adds CW edges, then deletes CCW ones
                // The reverse is done here,
                // Reason is because when an edge is added with a ref distance value "d" already exists in the tree
                // Removing the older "d" also removes the newly added one
                foreach (var edge in eventPoint.GetEdgesOnCCwSide(v))
                {
                    if (edge != null)
                    {
                        _bst.Delete(edge);
                    }
                }
                foreach (var edge in eventPoint.GetEdgesOnCwSide(v))
                {
                    if (edge != null)
                    {
                        edge.DistanceToReference = edge.DistanceTo(v.Position); // TODO: This line smells
                        _bst.Add(edge);
                    }
                }

            }

            _bst.Clear();
            return visiblePoints;
        }

        private bool IsVisible(Vertex from, Vertex to)
        {
            // Neighboring vertices are assumed to be seeing each other
            if (from.IsNeighborWith(to))
            {
                return true;
            }

            // Non-neighbor vertices of the same polygon don't see each other
            if (from.OwnerPolygon.Vertices.Contains(to))
            {
                return false;
            }

            // Check with if intersecting with owner polygon
            // Nudge a little bit away from polygon, so it won't intersect with neighboring edges
            var nudgedFrom = from.Position + (to.Position - from.Position).normalized * 0.0001f;
            if (from.OwnerPolygon.IntersectsWith(nudgedFrom.x, nudgedFrom.z, to.Position.x, to.Position.z))
            {
                return false;
            }

            Edge leftMostEdge;
            _bst.GetMin(out leftMostEdge);

            if (leftMostEdge != null
                && leftMostEdge.IntersectsWith(from.Position.x, from.Position.z, to.Position.x, to.Position.z))
            {
                return false;
            }

            return true;
        }

        public void Draw()
        {
            foreach (var pathEdge in _pathEdgeDict.Values)
            {
                Debug.DrawLine(pathEdge.Vertex1.Position, pathEdge.Vertex2.Position, Color.red);
            }
        }
    }
}
