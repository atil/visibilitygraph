using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using Pathfinding;
using UnityEngine.UI;

public class Test : MonoBehaviour
{
    public GameObject Obs;
    private Vector3[] floor;
    private Polygon p;

    public List<Vector3> points = new List<Vector3>();
    public GameObject textPrefab;
    public GameObject canvas;

    void Start ()
	{
        floor = GetFloor(Obs.GetComponent<Collider>().bounds);
        p = new Polygon(floor);

        for (int i = 0; i < 10; i++)
        {
            points.Add(new Vector3(Random.value * 20 - 10, 0, Random.value * 20 - 10));
            GameObject.CreatePrimitive(PrimitiveType.Sphere).transform.position = points[i];
        }

        points = Util.SortClockwise(Vector3.zero, points);
        GameObject.CreatePrimitive(PrimitiveType.Cube);

        var index = 0;
        foreach (Vector3 t in points)
        {
            var screenPos = Camera.main.WorldToScreenPoint(t);
            var text = Instantiate(textPrefab, screenPos, Quaternion.identity) as GameObject;
            text.transform.SetParent(canvas.transform);
            text.GetComponent<Text>().text = index.ToString();
            index++;
        }


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
