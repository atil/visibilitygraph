using System.Collections.Generic;
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
                RemoveEdgesOfVertex(touchingVertex);
            }

            foreach (var touchingVertex in touchingVertices)
            {
                CalculateVisiblityForVertex(touchingVertex, _allVertices);
            }
        }

        public void RemovePolygon(Polygon polygon)
        {
            // Remove the polygon's edges and recalculate the polygon's neighbors
            var touchingVertices = GetTouchingVertices(polygon);

            foreach (var vertex in polygon.Vertices)
            {
                RemoveEdgesOfVertex(vertex);
            }

            _polygons.Remove(polygon);
            foreach (var v in polygon.Vertices)
            {
                _allVertices.Remove(v);
            }

            foreach (var touchingVertex in touchingVertices)
            {
                CalculateVisiblityForVertex(touchingVertex, _allVertices);
            }

        }

        public void MovePolygon(Polygon polygon, Vector3 moveVec)
        {
            // Store the polygon's existing neighbors
            var touchingVertices = GetTouchingVertices(polygon);

            // Remove the polygon's edges
            foreach (var vertex in polygon.Vertices)
            {
                RemoveEdgesOfVertex(vertex);
            }

            // Move polygon and calculate visibility for it
            polygon.Move(moveVec);
            foreach (var vertex in polygon.Vertices)
            {
                CalculateVisiblityForVertex(vertex, _allVertices);
            }

            // Append the polygon's new neighbors
            foreach (var touchingVertex in GetTouchingVertices(polygon))
            {
                touchingVertices.Add(touchingVertex);
            }

            // Recalculate touching vertices
            foreach (var touchingVertex in touchingVertices)
            {
                RemoveEdgesOfVertex(touchingVertex);
            }

            foreach (var touchingVertex in touchingVertices)
            {
                CalculateVisiblityForVertex(touchingVertex, _allVertices);
            }
        }

        private void RemoveEdgesOfVertex(Vertex vertex)
        {
            // Remove the edges of which the vertex is element of
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

        private HashSet<Vertex> GetTouchingVertices(Polygon polygon)
        {
            var retVal = new HashSet<Vertex>();
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

        #region Shortest path
        private Vertex GetClosestVertexTo(Vector3 p)
        {
            var minDistSqr = float.MaxValue;
            var closestVertex = _allVertices[0];

            foreach (var vertex in _allVertices)
            {
                var distSqr = (vertex.Position - p).sqrMagnitude;
                if (distSqr < minDistSqr)
                {
                    closestVertex = vertex;
                    minDistSqr = distSqr;
                }
            }

            return closestVertex;
        }

        public Vector3[] GetPath(Vector3 srcPos, Vector3 destPos)
        {
            if (_allVertices.Count < 1 // No vertices in graph
                || !IntersectsWith(srcPos.x, srcPos.z, destPos.x, destPos.z)) // Doesn't intersect with any polygon
            {
                return new[] { srcPos, destPos };
            }

            var srcVertex = GetClosestVertexTo(srcPos);
            var destVertex = GetClosestVertexTo(destPos);

            if (srcVertex == destVertex) // Don't know this will ever be true
            {
                return new[] { srcPos, destPos };
            }

            // Here be Djikstra
            var allVertsCopy = new HashSet<Vertex>(_allVertices);

            var distances = new Dictionary<Vertex, float>();
            var prevs = new Dictionary<Vertex, Vertex>();

            foreach (var vertex in allVertsCopy)
            {
                distances.Add(vertex, float.MaxValue);
                prevs.Add(vertex, null);
            }

            distances[srcVertex] = 0f;

            while (allVertsCopy.Count > 0)
            {
                Vertex u = null;
                var minDist = float.MaxValue;
                foreach (var vertex in allVertsCopy)
                {
                    if (distances[vertex] < minDist)
                    {
                        minDist = distances[vertex];
                        u = vertex;
                    }
                }

                if (u == destVertex) // Destination found
                {
                    break;
                }

                allVertsCopy.Remove(u);

                Debug.Assert(u != null);
                var neighbors = GetNeighbors(u, allVertsCopy);
                foreach (var v in neighbors)
                {
                    // TODO: PathEdge's Weight should be used instead of Distance() call
                    // No need to calculate it again
                    var altDist = distances[u] + Vector3.Distance(v.Position, u.Position); 
                    if (altDist < distances[v])
                    {
                        distances[v] = altDist;
                        prevs[v] = u;
                    }
                }
            }

            var path = new List<Vector3>();
            var w = destVertex;

            while (prevs[w] != null)
            {
                path.Insert(0, w.Position);
                w = prevs[w];
            }
            path.Insert(0, w.Position);
            
            return SimplifyPath(srcPos, destPos, path);
        }

        public List<Vertex> GetNeighbors(Vertex vertex, HashSet<Vertex> allVertsCopy)
        {
            var retVal = new List<Vertex>();
            foreach (var pathEdge in _pathEdgeDict.Values)
            {
                if (pathEdge.Vertex1 == vertex
                    && allVertsCopy.Contains(pathEdge.Vertex2))
                {
                    retVal.Add(pathEdge.Vertex2);
                }
                if (pathEdge.Vertex2 == vertex
                    && allVertsCopy.Contains(pathEdge.Vertex1))
                {
                    retVal.Add(pathEdge.Vertex1);
                }
            }
            return retVal;
        }

        public Vector3[] SimplifyPath(Vector3 src, Vector3 dest, List<Vector3> path)
        {
            if (path.Count > 2) // TODO: Not sure about this check
            {
                int iSrc = -1, iDest = -1;
                for (var i = 1; i < path.Count; i++)
                {
                    if (IntersectsWith(src.x, src.z, path[i].x, path[i].z))
                    {
                        iSrc = i;
                        break;
                    }
                }

                for (int i = path.Count - 1, j = 0; i >= 0; i--, j++)
                {
                    if (IntersectsWith(dest.x, dest.z, path[i].x, path[i].z))
                    {
                        break;
                    }
                    iDest = j;
                }

                for (var i = 0; i < iSrc - 1; i++)
                {
                    path.RemoveAt(0);
                }

                for (var i = 0; i < iDest; i++)
                {
                    path.RemoveAt(path.Count - 1);
                }

            }

            path.Insert(0, src);
            path.Add(dest);

            return path.ToArray();
        }

        private bool IntersectsWith(float v1X, float v1Z, float v2X, float v2Z)
        {
            foreach (var polygon in _polygons)
            {
                if (polygon.IntersectsWith(v1X, v1Z, v2X, v2Z))
                {
                    return true;
                }
            }

            return false;
        }

        #endregion

        public void Draw()
        {
            foreach (var pathEdge in _pathEdgeDict.Values)
            {
                Debug.DrawLine(pathEdge.Vertex1.Position, pathEdge.Vertex2.Position, Color.red);
            }
        }
    }
}
