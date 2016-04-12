using UnityEngine;
using Navigation;

public class TestAgentView : MonoBehaviour
{
    public IAgent Agent;

    private Transform _thisTransform;

    void Awake()
    {
        _thisTransform = transform;
        Agent = new Agent(GetComponent<BoxCollider>().bounds, _thisTransform.position);
        GetComponent<Renderer>().enabled = false;
    }

    public void MyUpdate()
    {
        Agent.Update(_thisTransform.position);
    }
}
