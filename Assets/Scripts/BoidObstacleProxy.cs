using Unity.Entities;
using UnityEngine;

public struct BoidObstacle : IComponentData
{
}

[DisallowMultipleComponent]
public class BoidObstacleProxy : ComponentDataProxy<BoidObstacle>
{
}