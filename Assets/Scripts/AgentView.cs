using UnityEngine;
using Navigation;

public class AgentView : MonoBehaviour
{
    public IAgent Agent;

    private Transform _thisTransform;

    void Awake()
    {
        _thisTransform = transform;
    }

    void Update()
    {
        Agent.SetPosition(_thisTransform.position);
    }

    private Vector3[] GetFloor(Bounds b)
    {
        var bMin = b.min;
        var bMax = b.max;
        var b1 = new Vector3(bMin.x, 0, bMin.z);
        var b2 = new Vector3(bMax.x, 0, bMin.z);
        var b3 = new Vector3(bMax.x, 0, bMax.z);
        var b4 = new Vector3(bMin.x, 0, bMax.z);

        return new[] { b1, b2, b3, b4 };
    }
}
