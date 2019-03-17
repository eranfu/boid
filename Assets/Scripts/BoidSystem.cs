using System.Collections.Generic;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateBefore(typeof(TransformSystemGroup))]
public class BoidSystem : JobComponentSystem
{
    #region Inner Defines

    private struct PrevCells
    {
        public NativeMultiHashMap<int, int> hashMap;
        public NativeArray<int> cellIndex;
        public NativeArray<float3> targetPositions;
        public NativeArray<float3> obstaclePositions;
        public NativeArray<float3> cellAlignment;
        public NativeArray<float3> cellSeparation;
        public NativeArray<int> cellNearestObstacleIndex;
        public NativeArray<float> cellNearestObstacleDistance;
        public NativeArray<int> cellNearestTargetIndex;
        public NativeArray<int> cellCount;
    }

    [BurstCompile]
    private struct JobCopyHeadings : IJobProcessComponentDataWithEntity<LocalToWorld>
    {
        public NativeArray<float3> headings;

        public void Execute(Entity entity, int index, [ReadOnly] ref LocalToWorld localToWorld)
        {
            headings[index] = localToWorld.Forward;
        }
    }

    [BurstCompile]
    private struct JobCopyPosition : IJobProcessComponentDataWithEntity<LocalToWorld>
    {
        public NativeArray<float3> positions;

        public void Execute(Entity entity, int index, [ReadOnly] ref LocalToWorld localToWorld)
        {
            positions[index] = localToWorld.Position;
        }
    }

    [BurstCompile]
    [RequireComponentTag(typeof(Boid))]
    private struct JobHashPositions : IJobProcessComponentDataWithEntity<LocalToWorld>
    {
        public NativeMultiHashMap<int, int>.Concurrent hashMap;
        [ReadOnly] public float cellRadius;

        public void Execute(Entity entity, int index, [ReadOnly] ref LocalToWorld localToWorld)
        {
            var hash = (int) math.hash(math.int3(math.floor(localToWorld.Position / cellRadius)));
            hashMap.Add(hash, index);
        }
    }

    [BurstCompile]
    private struct JobMergeCells : IJobNativeMultiHashMapMergedSharedKeyIndices
    {
        [ReadOnly] public NativeArray<float3> obstaclePositions;
        [ReadOnly] public NativeArray<float3> targetPositions;
        public NativeArray<float3> cellSeparation;
        public NativeArray<int> cellCount;
        public NativeArray<int> cellNearestObstacleIndex;
        public NativeArray<float> cellNearestObstacleDistance;
        public NativeArray<int> cellNearestTargetIndex;
        public NativeArray<int> cellIndex;
        public NativeArray<float3> cellAlignment;

        private void NearestPosition(
            NativeArray<float3> targets, float3 position, out int nearestPositionIndex, out float nearestDistance)
        {
            nearestPositionIndex = 0;
            nearestDistance = math.lengthsq(position - targets[0]);
            for (var i = 1; i < targets.Length; i++)
            {
                float3 target = targets[i];
                float distance = math.lengthsq(position - target);
                bool nearest = distance < nearestDistance;
                nearestDistance = math.select(nearestDistance, distance, nearest);
                nearestPositionIndex = math.select(nearestPositionIndex, i, nearest);
            }

            nearestDistance = math.sqrt(nearestDistance);
        }

        public void ExecuteFirst(int index)
        {
            float3 position = cellSeparation[index] / cellCount[index];

            NearestPosition(obstaclePositions, position, out int nearestObstacleIndex,
                out float nearestObstacleDistance);
            cellNearestObstacleIndex[index] = nearestObstacleIndex;
            cellNearestObstacleDistance[index] = nearestObstacleDistance;

            NearestPosition(targetPositions, position, out int nearestTargetIndex, out _);
            cellNearestTargetIndex[index] = nearestTargetIndex;

            cellIndex[index] = index;
        }

        public void ExecuteNext(int ci, int index)
        {
            cellCount[ci] += 1;
            cellAlignment[ci] = cellAlignment[ci] + cellAlignment[index];
            cellSeparation[ci] = cellSeparation[ci] + cellSeparation[index];
            this.cellIndex[index] = ci;
        }
    }

    [BurstCompile]
    [RequireComponentTag(typeof(Boid))]
    private struct JobSteer : IJobProcessComponentDataWithEntity<LocalToWorld>
    {
        [ReadOnly] public Boid sharedBoid;
        [ReadOnly] public NativeArray<float3> obstaclePositions;
        [ReadOnly] public NativeArray<float3> targetPositions;
        [ReadOnly] public NativeArray<int> cellIndex;
        [ReadOnly] public NativeArray<int> cellNearestObstacleIndex;
        [ReadOnly] public NativeArray<float> cellNearestObstacleDistance;
        [ReadOnly] public NativeArray<int> cellNearestTargetIndex;
        [ReadOnly] public NativeArray<int> cellCount;
        [ReadOnly] public NativeArray<float3> cellSeparation;
        [ReadOnly] public NativeArray<float3> cellAlignment;
        [ReadOnly] public float dt;

        public void Execute(Entity entity, int index, ref LocalToWorld localToWorld)
        {
            float3 forward = localToWorld.Forward;
            float3 position = localToWorld.Position;
            int ci = cellIndex[index];
            int nearestObstacleIndex = cellNearestObstacleIndex[ci];
            float nearestObstacleDistance = cellNearestObstacleDistance[ci];
            int nearestTargetIndex = cellNearestTargetIndex[ci];
            int count = cellCount[ci];
            float3 separation = cellSeparation[ci];
            float3 alignment = cellAlignment[ci];
            float3 nearestObstaclePosition = obstaclePositions[nearestObstacleIndex];
            float3 nearestTargetPosition = targetPositions[nearestTargetIndex];

            float3 obstacleSteering = position - nearestObstaclePosition;
            float3 avoidObstacleHeading =
                (nearestObstaclePosition + math.normalizesafe(obstacleSteering) * sharedBoid.obstacleAversionDistance) -
                position;
            float3 targetHeading = sharedBoid.targetWeight * math.normalizesafe(nearestTargetPosition - position);
            float3 alignmentResult = sharedBoid.alignmentWeight * math.normalizesafe(alignment / count - forward);
            float3 separationResult = sharedBoid.separationWeight * math.normalizesafe(position * count - separation);
            float3 normalHeading = math.normalizesafe(targetHeading + alignmentResult + separationResult);
            float nearestObstacleDistanceFromRadius = nearestObstacleDistance - sharedBoid.obstacleAversionDistance;
            float3 targetForward =
                math.select(normalHeading, avoidObstacleHeading, nearestObstacleDistanceFromRadius < 0);
            float3 nextHeading = math.normalizesafe(forward + dt * (targetForward - forward));

            localToWorld = new LocalToWorld
            {
                Value = float4x4.TRS(position + nextHeading * dt * sharedBoid.moveSpeed,
                    quaternion.LookRotationSafe(nextHeading, math.up()),
                    new float3(1, 1, 1))
            };
        }
    }

    #endregion

    private ComponentGroup _boidGroup;
    private ComponentGroup _targetGroup;
    private ComponentGroup _obstacleGroup;
    private readonly List<PrevCells> _prevCells = new List<PrevCells>();
    private readonly List<Boid> _sharedBoids = new List<Boid>(10);

    protected override void OnCreateManager()
    {
        _boidGroup = GetComponentGroup(new EntityArchetypeQuery
        {
            All = new[] {ComponentType.ReadOnly<Boid>(), ComponentType.ReadWrite<LocalToWorld>()},
            Options = EntityArchetypeQueryOptions.FilterWriteGroup
        });

        _targetGroup = GetComponentGroup(new EntityArchetypeQuery
        {
            All = new[] {ComponentType.ReadOnly<LocalToWorld>(), ComponentType.ReadOnly<BoidTarget>()}
        });

        _obstacleGroup = GetComponentGroup(new EntityArchetypeQuery
        {
            All = new[] {ComponentType.ReadOnly<LocalToWorld>(), ComponentType.ReadOnly<BoidObstacle>()}
        });
    }

    protected override void OnStopRunning()
    {
        for (var i = 0; i < _prevCells.Count; i++)
        {
            _prevCells[i].hashMap.Dispose();
            _prevCells[i].cellIndex.Dispose();
            _prevCells[i].targetPositions.Dispose();
            _prevCells[i].obstaclePositions.Dispose();
            _prevCells[i].cellAlignment.Dispose();
            _prevCells[i].cellSeparation.Dispose();
            _prevCells[i].cellNearestObstacleIndex.Dispose();
            _prevCells[i].cellNearestObstacleDistance.Dispose();
            _prevCells[i].cellNearestTargetIndex.Dispose();
            _prevCells[i].cellCount.Dispose();
        }

        _prevCells.Clear();
    }

    protected override JobHandle OnUpdate(JobHandle inputDeps)
    {
        EntityManager.GetAllUniqueSharedComponentData(_sharedBoids);
        int obstacleCount = _obstacleGroup.CalculateLength();
        int targetCount = _targetGroup.CalculateLength();
        for (var sharedBoidIndex = 1; sharedBoidIndex < _sharedBoids.Count; sharedBoidIndex++)
        {
            Boid sharedBoid = _sharedBoids[sharedBoidIndex];
            _boidGroup.SetFilter(sharedBoid);
            int boidInstanceCount = _boidGroup.CalculateLength();
            int cacheIndex = sharedBoidIndex - 1;

            var nextCells = new PrevCells
            {
                hashMap = new NativeMultiHashMap<int, int>(boidInstanceCount, Allocator.TempJob),
                cellIndex = new NativeArray<int>(boidInstanceCount,
                    Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                targetPositions = new NativeArray<float3>(targetCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                obstaclePositions = new NativeArray<float3>(obstacleCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                cellAlignment = new NativeArray<float3>(boidInstanceCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                cellSeparation = new NativeArray<float3>(boidInstanceCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                cellNearestObstacleIndex = new NativeArray<int>(boidInstanceCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                cellNearestObstacleDistance = new NativeArray<float>(boidInstanceCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                cellNearestTargetIndex = new NativeArray<int>(boidInstanceCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
                cellCount = new NativeArray<int>(boidInstanceCount, Allocator.TempJob,
                    NativeArrayOptions.UninitializedMemory),
            };

            if (cacheIndex > _prevCells.Count - 1)
            {
                _prevCells.Add(nextCells);
            }
            else
            {
                _prevCells[cacheIndex].hashMap.Dispose();
                _prevCells[cacheIndex].cellIndex.Dispose();
                _prevCells[cacheIndex].targetPositions.Dispose();
                _prevCells[cacheIndex].obstaclePositions.Dispose();
                _prevCells[cacheIndex].cellAlignment.Dispose();
                _prevCells[cacheIndex].cellSeparation.Dispose();
                _prevCells[cacheIndex].cellNearestObstacleIndex.Dispose();
                _prevCells[cacheIndex].cellNearestObstacleDistance.Dispose();
                _prevCells[cacheIndex].cellNearestTargetIndex.Dispose();
                _prevCells[cacheIndex].cellCount.Dispose();

                _prevCells[cacheIndex] = nextCells;
            }

            var initialCellAlignmentJob = new JobCopyHeadings
            {
                headings = nextCells.cellAlignment
            };
            JobHandle initialCellAlignmentJobHandle = initialCellAlignmentJob.ScheduleGroup(_boidGroup, inputDeps);

            var initialCellSeparationJob = new JobCopyPosition
            {
                positions = nextCells.cellSeparation
            };
            JobHandle initialCellSeparationJobHandle = initialCellSeparationJob.ScheduleGroup(_boidGroup, inputDeps);

            var copyTargetPositionsJob = new JobCopyPosition
            {
                positions = nextCells.targetPositions
            };
            JobHandle copyTargetPositionsJobHandle = copyTargetPositionsJob.ScheduleGroup(_targetGroup, inputDeps);

            var copyObstaclePositionsJob = new JobCopyPosition
            {
                positions = nextCells.obstaclePositions
            };
            JobHandle copyObstaclePositionsJobHandle =
                copyObstaclePositionsJob.ScheduleGroup(_obstacleGroup, inputDeps);

            var hashPositionsJob = new JobHashPositions
            {
                cellRadius = sharedBoid.cellRadius,
                hashMap = nextCells.hashMap.ToConcurrent()
            };
            JobHandle hashPositionsJobHandle = hashPositionsJob.ScheduleGroup(_boidGroup, inputDeps);

            var initialCellCountJob = new MemsetNativeArray<int>
            {
                Source = nextCells.cellCount,
                Value = 1
            };
            JobHandle initialCellCountJobHandle =
                initialCellCountJob.Schedule(nextCells.cellCount.Length, 64, inputDeps);

            JobHandle initialCellBarrierJobHandle = JobHandle.CombineDependencies(
                initialCellCountJobHandle, initialCellAlignmentJobHandle, initialCellSeparationJobHandle);
            JobHandle copyTargetObstacleBarrierJobHandle =
                JobHandle.CombineDependencies(copyTargetPositionsJobHandle, copyObstaclePositionsJobHandle);
            JobHandle mergeCellsBarrierJobHandle = JobHandle.CombineDependencies(
                hashPositionsJobHandle, initialCellBarrierJobHandle, copyTargetObstacleBarrierJobHandle);

            var mergeCellsJob = new JobMergeCells
            {
                cellAlignment = nextCells.cellAlignment,
                cellCount = nextCells.cellCount,
                cellIndex = nextCells.cellIndex,
                cellNearestObstacleDistance = nextCells.cellNearestObstacleDistance,
                cellNearestObstacleIndex = nextCells.cellNearestObstacleIndex,
                cellNearestTargetIndex = nextCells.cellNearestTargetIndex,
                cellSeparation = nextCells.cellSeparation,
                obstaclePositions = nextCells.obstaclePositions,
                targetPositions = nextCells.targetPositions,
            };
            JobHandle mergeCellsJobHandle = mergeCellsJob.Schedule(nextCells.hashMap, 64, mergeCellsBarrierJobHandle);

            var steerJob = new JobSteer
            {
                sharedBoid = sharedBoid,
                cellAlignment = nextCells.cellAlignment,
                cellCount = nextCells.cellCount,
                cellIndex = nextCells.cellIndex,
                cellNearestObstacleDistance = nextCells.cellNearestObstacleDistance,
                cellNearestObstacleIndex = nextCells.cellNearestObstacleIndex,
                cellNearestTargetIndex = nextCells.cellNearestTargetIndex,
                cellSeparation = nextCells.cellSeparation,
                obstaclePositions = nextCells.obstaclePositions,
                targetPositions = nextCells.targetPositions,
                dt = Time.deltaTime,
            };
            JobHandle steerJobHandle = steerJob.ScheduleGroup(_boidGroup, mergeCellsJobHandle);
            inputDeps = steerJobHandle;
            _boidGroup.AddDependency(inputDeps);
        }

        _sharedBoids.Clear();
        return inputDeps;
    }
}