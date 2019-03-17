using System;
using Unity.Entities;
using Unity.Transforms;

[Serializable]
[WriteGroup(typeof(LocalToWorld))]
public struct Boid : ISharedComponentData
{
    public float cellRadius;
    public float separationWeight;
    public float alignmentWeight;
    public float targetWeight;
    public float obstacleAversionDistance;
    public float moveSpeed;
}

public class BoidProxy : SharedComponentDataProxy<Boid>
{
}