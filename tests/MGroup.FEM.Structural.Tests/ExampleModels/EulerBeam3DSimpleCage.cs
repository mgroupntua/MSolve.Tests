using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.FEM.Structural.Line;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Line;
using MGroup.Constitutive.Structural.BoundaryConditions;
using System.Linq;
using MGroup.MSolve.Discretization.BoundaryConditions;
using System;
using System.IO;
using MGroup.Constitutive.Structural.InitialConditions;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class EulerBeam3DSimpleCage
	{
		public static readonly double expected_solution7 = 148.936792350562;
        public static Dictionary<int, double[]> sensorData;
        public static double timeOffset;

        public static Dictionary<int, double[]> CreateDictionaryOfSensorData(string filename) => File.ReadAllLines(filename).Skip(1)
            .Select((x, index) => x.Split(',').Length > 0 ? x.Split(',').Select(v => Convert.ToDouble(v)).ToArray() : new double[] { -index })
            .GroupBy(x => (int)(1000 * x[0]))
            .ToDictionary(x => x.Key, x => x.FirstOrDefault());

        // all accelerations as cm/sec^2
        private static double AccelerationX(double time, double amount) => sensorData.ContainsKey((int)((time + timeOffset) * 1000)) ? sensorData[(int)((time + timeOffset) * 1000)][4] * 100 : 0;
        private static double AccelerationY(double time, double amount) => sensorData.ContainsKey((int)((time + timeOffset) * 1000)) ? sensorData[(int)((time + timeOffset) * 1000)][5] * 100 : 0;
        private static double AccelerationZ(double time, double amount) => sensorData.ContainsKey((int)((time + timeOffset) * 1000)) ? sensorData[(int)((time + timeOffset) * 1000)][6] * 100 : 0;
        private static double AccelerationFL(double time, double amount) => sensorData.ContainsKey((int)((time + timeOffset) * 1000)) ? sensorData[(int)((time + timeOffset) * 1000)][7] * 100 : 0;
        private static double AccelerationFR(double time, double amount) => sensorData.ContainsKey((int)((time + timeOffset) * 1000)) ? sensorData[(int)((time + timeOffset) * 1000)][8] * 100 : 0;
        private static double AccelerationRL(double time, double amount) => sensorData.ContainsKey((int)((time + timeOffset) * 1000)) ? sensorData[(int)((time + timeOffset) * 1000)][9] * 100 : 0;
        private static double AccelerationRR(double time, double amount) => sensorData.ContainsKey((int)((time + timeOffset) * 1000)) ? sensorData[(int)((time + timeOffset) * 1000)][10] * 100 : 0;

        public static Model CreateModel()
        {
            //sensorData = CreateDictionaryOfSensorData(@"D:\RacecarData\Goat.csv");
            string path = (Environment.GetEnvironmentVariable("SYSTEM_DEFINITIONID") != null)
                ? @"..\..\Data\" : @"..\..\..\Data\";
            sensorData = CreateDictionaryOfSensorData(path + "SensorData.csv");
            timeOffset = sensorData.Select(x => x.Key).Where(x => x > 0).Min() * 0.001;
            // Units: Length: cm, Stress: KN/cm2, Density: ton/cm3, Mass (of data array below): kg
            var modelData = new[]
            {
                new[] { 10d, 20d, 120d, 23d },
                new[] { 20d, 30d, 120d, 23d },
                new[] { 10d, 40d, 67.5d, 82d },
                new[] { 20d, 50d, 67.5d, 164d },
                new[] { 30d, 60d, 67.5d, 31.5d },
                new[] { 40d, 50d, 120d, 180d },
                new[] { 50d, 60d, 120d, 145d },
                new[] { 40d, 70d, 67.5d, 52d },
                new[] { 50d, 80d, 67.5d, 164d },
                new[] { 60d, 90d, 67.5d, 31.5d },
                new[] { 70d, 80d, 120d, 23d },
                new[] { 80d, 90d, 120d, 23d },
                new[] { 05d, 10d, 50d, 10d },
                new[] { 25d, 30d, 50d, 10d },
                new[] { 65d, 70d, 50d, 10d },
                new[] { 85d, 90d, 50d, 10d },
            };

            var model = new Model();
            model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));
            var nodes = new[]
            {
                new Node(id: 05, x: 0d, y: 0d, z: -50d),
                new Node(id: 10, x: 0d, y: 0d, z: 0d),
                new Node(id: 20, x: 120d, y: 0d, z: 0d),
                new Node(id: 25, x: 240d, y: 0d, z: -50d),
                new Node(id: 30, x: 240d, y: 0d, z: 0d),
                new Node(id: 40, x: 0d, y: 67.5d, z: 0d),
                new Node(id: 50, x: 120d, y: 67.5d, z: 0d),
                new Node(id: 60, x: 240d, y: 67.5d, z: 0d),
                new Node(id: 65, x: 0d, y: 135d, z: -50d),
                new Node(id: 70, x: 0d, y: 135d, z: 0d),
                new Node(id: 80, x: 120d, y: 135d, z: 0d),
                new Node(id: 85, x: 240, y: 135d, z: -50d),
                new Node(id: 90, x: 240, y: 135d, z: 0d),
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

            var elements = modelData.Select((x, index) => index < 12 ? new EulerBeam3D(new[] { model.NodesDictionary[(int)x[0]], model.NodesDictionary[(int)x[1]] }, 2.1, 0.3)
            {
                ID = index + 1,
                SectionArea = 91.04,
                MomentOfInertiaY = 2843,
                MomentOfInertiaZ = 8091,
                MomentOfInertiaPolar = 76.57,
                SubdomainID = 0,
                Density = 1e-3 * x[3] / (x[2] * 91.04),
            } :
            new EulerBeam3D(new[] { model.NodesDictionary[(int)x[0]], model.NodesDictionary[(int)x[1]] }, 0.0549, 0.3)
            {
                ID = index + 1,
                SectionArea = 91.04,
                MomentOfInertiaY = 2843,
                MomentOfInertiaZ = 8091,
                MomentOfInertiaPolar = 76.57,
                SubdomainID = 0,
                Density = 1e-3 * x[3] / (x[2] * 91.04),
            });

            foreach (var element in elements)
            {
                model.ElementsDictionary.Add(element.ID, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
            }

            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
                new[]
                {
                    new NodalDisplacement(model.NodesDictionary[05], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[05], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[05], StructuralDof.TranslationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[05], StructuralDof.RotationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[05], StructuralDof.RotationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[05], StructuralDof.RotationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[25], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[25], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[25], StructuralDof.TranslationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[65], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[65], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[65], StructuralDof.TranslationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[65], StructuralDof.RotationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[65], StructuralDof.RotationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[65], StructuralDof.RotationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[85], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[85], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[85], StructuralDof.TranslationZ, amount: 0d),
                },
                Array.Empty<NodalLoad>()
            ));

            var accelerationsX = nodes.Where(x => x.ID % 10 == 0).Select(x => new NodalAcceleration(x, StructuralDof.TranslationX, 1d)).ToArray();
            var accelerationsY = nodes.Where(x => x.ID % 10 == 0).Select(x => new NodalAcceleration(x, StructuralDof.TranslationY, 1d)).ToArray();
            var accelerationsZ = nodes.Where(x => x.ID % 10 == 0 && x.ID != 10 && x.ID != 30 && x.ID != 70 && x.ID != 90).Select(x => new NodalAcceleration(x, StructuralDof.TranslationZ, 1d)).ToArray();
            var accelerationFL = new NodalAcceleration(model.NodesDictionary[90], StructuralDof.TranslationZ, 1d);
            var accelerationFR = new NodalAcceleration(model.NodesDictionary[30], StructuralDof.TranslationZ, 1d);
            var accelerationRL = new NodalAcceleration(model.NodesDictionary[70], StructuralDof.TranslationZ, 1d);
            var accelerationRR = new NodalAcceleration(model.NodesDictionary[10], StructuralDof.TranslationZ, 1d);

            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet(new[] { new StructuralBoundaryConditionSet(accelerationsX, Array.Empty<INodalLoadBoundaryCondition>()) }.ToList(), AccelerationX));
            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet(new[] { new StructuralBoundaryConditionSet(accelerationsY, Array.Empty<INodalLoadBoundaryCondition>()) }.ToList(), AccelerationY));
            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet(new[] { new StructuralBoundaryConditionSet(accelerationsZ, Array.Empty<INodalLoadBoundaryCondition>()) }.ToList(), AccelerationZ));
            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet(new[] { new StructuralBoundaryConditionSet(new[] { accelerationFL }, Array.Empty<INodalLoadBoundaryCondition>()) }.ToList(), AccelerationFL));
            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet(new[] { new StructuralBoundaryConditionSet(new[] { accelerationFR }, Array.Empty<INodalLoadBoundaryCondition>()) }.ToList(), AccelerationFR));
            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet(new[] { new StructuralBoundaryConditionSet(new[] { accelerationRL }, Array.Empty<INodalLoadBoundaryCondition>()) }.ToList(), AccelerationRL));
            model.BoundaryConditions.Add(new StructuralTransientBoundaryConditionSet(new[] { new StructuralBoundaryConditionSet(new[] { accelerationRR }, Array.Empty<INodalLoadBoundaryCondition>()) }.ToList(), AccelerationRR));

            return model;
		}
	}
}
