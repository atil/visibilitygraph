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
    }

    void Update()
    {
        Agent.Update(_thisTransform.position);
    }
}
