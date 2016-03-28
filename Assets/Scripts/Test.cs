using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using Pathfinding.SelfBalancedTree;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class Test : MonoBehaviour
{
    private readonly List<Polygon> _polygons = new List<Polygon>();
    private readonly List<Vertex> _visiblePoints = new List<Vertex>();
    private readonly AVLTree<Edge> _bst = new AVLTree<Edge>();

    public Transform Reference;

    void Start ()
    {
        foreach (var obstacleGo in GameObject.FindGameObjectsWithTag("Obstacle"))
        {
            var floor = GetFloor(obstacleGo.GetComponent<Collider>().bounds);
            _polygons.Add(new Polygon(floor));
        }

    }

    private bool IsVisible(Vector3 from, Vector3 to)
    {
        return true;
    }

    private List<Vertex> GetEventPoints(List<Polygon> polys)
    {
        var retVal = new List<Vertex>();
        foreach (var polygon in polys)
        {
            foreach (var v in polygon.Vertices)
            {
                retVal.Add(v);
            }
        }
        return retVal;
    }

    void VisibilePoints()
    {
        _visiblePoints.Clear();
        var events = GetEventPoints(_polygons);
        var sortedEvents = Util.SortClockwise(Vector3.zero, events);

        var ray = new Ray(Reference.position, Vector3.right);
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
            // TODO: All edges' DistanceToRef params might be needed to be updated here

            ray = new Ray(Reference.position, eventPoint.Position);

            if (IsVisible(Reference.position, eventPoint.Position))
            {

                _visiblePoints.Add(eventPoint);
                foreach (var edge in eventPoint.GetEdgesOnSide(true))
                {
                    float t;
                    var succ = edge.IntersectsWith(ray, out t);

                    // Error checking
                    if (!succ)
                    {
                        Debug.LogError("Something's wrong! Added edge should intersect with rotationally sweeping plane");
                        Debug.LogError("Edge with points : " + edge.Vertex1.Position + " and " + edge.Vertex2.Position);
#if UNITY_EDITOR
                        EditorApplication.isPlaying = false;
#endif
                    }

                    edge.DistanceToReference = t;

                    _bst.Add(edge);
                }

                foreach (var edge in eventPoint.GetEdgesOnSide(false))
                {
                    _bst.Delete(edge);
                }
            }
        }

        _bst.Clear();

        foreach (var visiblePoint in _visiblePoints)
        {
            Debug.DrawLine(Reference.position, visiblePoint.Position);
        }
    }

    void Update()
    {
        VisibilePoints();
    }

    private Vector3[] GetFloor(Bounds b)
    {
        var bMin = b.min;
        var bMax = b.max;
        var b1 = new Vector3(bMin.x, bMin.y, bMin.z);
        var b2 = new Vector3(bMax.x, bMin.y, bMin.z);
        var b3 = new Vector3(bMax.x, bMin.y, bMax.z);
        var b4 = new Vector3(bMin.x, bMin.y, bMax.z);

        return new[] {b1, b2, b3, b4};
    }

}
