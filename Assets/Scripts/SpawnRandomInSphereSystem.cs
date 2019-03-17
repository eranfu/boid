using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(TransformSystemGroup))]
public class SpawnRandomInSphereSystem : ComponentSystem
{
    private struct SpawnerInstance
    {
        public Entity entity;
        public float3 center;
        public int spawnerIndex;
    }

    private ComponentGroup _mainGroup;

    protected override void OnCreateManager()
    {
        _mainGroup = GetComponentGroup(ComponentType.ReadOnly<SpawnRandomInSphere>(),
            ComponentType.ReadOnly<LocalToWorld>());
    }

    protected override void OnUpdate()
    {
        var spawnerList = new List<SpawnRandomInSphere>(10);
        EntityManager.GetAllUniqueSharedComponentData(spawnerList);
        var spawnInstanceCount = 0;
        foreach (SpawnRandomInSphere spawner in spawnerList)
        {
            _mainGroup.SetFilter(spawner);
            int calculateLength = _mainGroup.CalculateLength();
            spawnInstanceCount += calculateLength;
        }

        if (spawnInstanceCount == 0)
            return;

        var instances = new NativeArray<SpawnerInstance>(spawnInstanceCount, Allocator.Temp);

        var instanceIndex = 0;
        for (var spawnerIndex = 0; spawnerIndex < spawnerList.Count; spawnerIndex++)
        {
            SpawnRandomInSphere spawner = spawnerList[spawnerIndex];
            _mainGroup.SetFilter(spawner);
            int count = _mainGroup.CalculateLength();
            if (count == 0)
                continue;

            NativeArray<Entity> entities = _mainGroup.ToEntityArray(Allocator.TempJob);
            NativeArray<LocalToWorld> translations =
                _mainGroup.ToComponentDataArray<LocalToWorld>(Allocator.TempJob);

            for (var i = 0; i < count; i++)
            {
                var instance = new SpawnerInstance
                {
                    spawnerIndex = spawnerIndex,
                    center = translations[i].Position,
                    entity = entities[i]
                };
                instances[instanceIndex] = instance;
                ++instanceIndex;
            }

            translations.Dispose();
            entities.Dispose();
        }


        for (var i = 0; i < spawnInstanceCount; i++)
        {
            SpawnerInstance instance = instances[i];
            SpawnRandomInSphere spawner = spawnerList[instance.spawnerIndex];
            var points = new NativeArray<float3>(spawner.count, Allocator.TempJob);
            GeneratePoints.RandomPointsInUnitSphere(points);
            var entities = new NativeArray<Entity>(spawner.count, Allocator.Temp);
            EntityManager.Instantiate(spawner.prefab, entities);

            for (var j = 0; j < spawner.count; j++)
            {
                EntityManager.SetComponentData(entities[j], new LocalToWorld
                {
                    Value = float4x4.TRS(
                        instance.center + points[j] * spawner.radius,
                        quaternion.LookRotationSafe(points[j], math.up()),
                        new float3(1, 1, 1))
                });
            }

            EntityManager.RemoveComponent<SpawnRandomInSphere>(instance.entity);

            points.Dispose();
            entities.Dispose();
        }

        instances.Dispose();
    }
}