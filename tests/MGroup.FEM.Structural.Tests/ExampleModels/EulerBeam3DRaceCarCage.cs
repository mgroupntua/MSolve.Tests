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
using MGroup.FEM.Structural.Special;
using System.Reflection.Metadata;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class EulerBeam3DRaceCarCage
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
                new[] { 10, 30 },
                new[] { 10, 50 },
                new[] { 30, 70 },
                new[] { 30, 90 },
                new[] { 50, 70 },
                new[] { 50, 110 },
                new[] { 50, 130 },
                new[] { 70, 90 },
                new[] { 70, 130 },
                new[] { 70, 150 },
                new[] { 110, 130 },
                new[] { 110, 190 },
                new[] { 130, 150 },
                new[] { 130, 210 },
                new[] { 150, 170 },
                new[] { 170, 210 },
                new[] { 190, 210 },
                new[] { 190, 230 },
                new[] { 210, 250 },
                new[] { 230, 250 },
                new[] { 10, 20 },
                new[] { 30, 40 },
                new[] { 50, 60 },
                new[] { 70, 80 },
                new[] { 90, 100 },
                new[] { 110, 120 },
                new[] { 130, 140 },
                new[] { 150, 160 },
                new[] { 170, 180 },
                new[] { 190, 200 },
                new[] { 210, 220 },
                new[] { 230, 240 },
                new[] { 250, 260 },
                new[] { 20, 40 },
                new[] { 20, 60 },
                new[] { 40, 80 },
                new[] { 40, 100 },
                new[] { 60, 80 },
                new[] { 60, 120 },
                new[] { 60, 140 },
                new[] { 80, 100 },
                new[] { 80, 140 },
                new[] { 80, 160 },
                new[] { 120, 140 },
                new[] { 120, 200 },
                new[] { 140, 160 },
                new[] { 140, 220 },
                new[] { 160, 180 },
                new[] { 180, 220 },
                new[] { 200, 220 },
                new[] { 200, 240 },
                new[] { 220, 260 },
                new[] { 240, 260 },
                new[] { 5, 10 },
                new[] { 15, 20 },
                new[] { 225, 230 },
                new[] { 235, 240 },
            };

            var model = new Model();
            model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));
            var nodes = new[]
            {
                new Node(id: 5, x: 0, y: 24, z: -10), 
                new Node(id: 10, x: 0, y: 24, z: 0),
                new Node(id: 15, x: 0, y: -24, z: -10),
                new Node(id: 20, x: 0, y: -24, z: 0),
                new Node(id: 30, x: 0, y: 24, z: 30),
                new Node(id: 40, x: 0, y: -24, z: 30),
                new Node(id: 50, x: 80, y: 4, z: 0),
                new Node(id: 60, x: 80, y: 24, z: 0),
                new Node(id: 70, x: 80, y: 4, z: 30),
                new Node(id: 80, x: 80, y: 24, z: 30),
                new Node(id: 90, x: 80, y: 8, z: 50),
                new Node(id: 100, x: 80, y: -18, z: 50),
                new Node(id: 110, x: 163, y: 36.4, z: 0),
                new Node(id: 120, x: 163, y: -36.4, z: 0),
                new Node(id: 130, x: 163, y: 36.4, z: 30),
                new Node(id: 140, x: 163, y: -36.4, z: 30),
                new Node(id: 150, x: 163, y: 36.4, z: 50),
                new Node(id: 160, x: 163, y: -36.4, z: 50),
                new Node(id: 170, x: 163, y: 3.6, z: 143),
                new Node(id: 180, x: 163, y: -3.6, z: 143),
                new Node(id: 190, x: 213, y: 20, z: 0),
                new Node(id: 200, x: 213, y: -20, z: 0),
                new Node(id: 210, x: 213, y: 20, z: 30),
                new Node(id: 220, x: 213, y: -20, z: 30),
                new Node(id: 225, x: 258, y: 20, z: -10),
                new Node(id: 230, x: 258, y: 20, z: 0),
                new Node(id: 235, x: 258, y: -20, z:-10),
                new Node(id: 240, x: 258, y: -20, z:0),
                new Node(id: 250, x: 258, y: 20, z: 30),
                new Node(id: 260, x: 258, y: -20, z:30),
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

            var elements = modelData.Select((x, index) => index < 34 ? new EulerBeam3D(new[] { model.NodesDictionary[(int)x[0]], model.NodesDictionary[(int)x[1]] }, 20, 0.29)
            {
                ID = index + 1,
                SectionArea = 91.04,
                MomentOfInertiaY = 2843,
                MomentOfInertiaZ = 8091,
                MomentOfInertiaPolar = 76.57,
                SubdomainID = 0,
                Density = 7.8e-6,
            } :
            new EulerBeam3D(new[] { model.NodesDictionary[(int)x[0]], model.NodesDictionary[(int)x[1]] }, 0.549, 0.3)
            {
                ID = index + 1,
                SectionArea = 91.04,
                MomentOfInertiaY = 2843,
                MomentOfInertiaZ = 8091,
                MomentOfInertiaPolar = 76.57,
                SubdomainID = 0,
                Density = 7.8e-6,
            });
            var concentratedElements = new[]
            {
                //new ConcentratedMass3D(model.NodesDictionary[10], 5e-4) { ID = 100 },
                //new ConcentratedMass3D(model.NodesDictionary[20], 5e-4) { ID = 101 },
                //new ConcentratedMass3D(model.NodesDictionary[50], 5e-4) { ID = 102 },
                //new ConcentratedMass3D(model.NodesDictionary[60], 5e-4) { ID = 103 },
                //new ConcentratedMass3D(model.NodesDictionary[110], 45e-4) { ID = 104 },
                //new ConcentratedMass3D(model.NodesDictionary[120], 45e-4) { ID = 105 },
                //new ConcentratedMass3D(model.NodesDictionary[130], 15e-4) { ID = 106 },
                //new ConcentratedMass3D(model.NodesDictionary[140], 15e-4) { ID = 107 },
                //new ConcentratedMass3D(model.NodesDictionary[190], 15e-4) { ID = 108 },
                //new ConcentratedMass3D(model.NodesDictionary[200], 15e-4) { ID = 109 },
                //new ConcentratedMass3D(model.NodesDictionary[210], 15e-4) { ID = 110 },
                //new ConcentratedMass3D(model.NodesDictionary[220], 15e-4) { ID = 111 },
                //new ConcentratedMass3D(model.NodesDictionary[250], 10e-4) { ID = 112 },
                //new ConcentratedMass3D(model.NodesDictionary[260], 10e-4) { ID = 113 },
                new ConcentratedMass3D(model.NodesDictionary[10], 2e-5) { ID = 100 },
                new ConcentratedMass3D(model.NodesDictionary[20], 2e-5) { ID = 101 },
                new ConcentratedMass3D(model.NodesDictionary[50], 2e-5) { ID = 102 },
                new ConcentratedMass3D(model.NodesDictionary[60], 2e-5) { ID = 103 },
                new ConcentratedMass3D(model.NodesDictionary[110], 20e-5) { ID = 104 },
                new ConcentratedMass3D(model.NodesDictionary[120], 20e-5) { ID = 105 },
                new ConcentratedMass3D(model.NodesDictionary[130], 7e-5) { ID = 106 },
                new ConcentratedMass3D(model.NodesDictionary[140], 7e-5) { ID = 107 },
                new ConcentratedMass3D(model.NodesDictionary[190], 7e-5) { ID = 108 },
                new ConcentratedMass3D(model.NodesDictionary[200], 7e-5) { ID = 109 },
                new ConcentratedMass3D(model.NodesDictionary[210], 7e-5) { ID = 110 },
                new ConcentratedMass3D(model.NodesDictionary[220], 7e-5) { ID = 111 },
                new ConcentratedMass3D(model.NodesDictionary[250], 5e-5) { ID = 112 },
                new ConcentratedMass3D(model.NodesDictionary[260], 5e-5) { ID = 113 },
            };

            foreach (var element in elements)
            {
                model.ElementsDictionary.Add(element.ID, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
            }

            foreach (var element in concentratedElements)
            {
                model.ElementsDictionary.Add(element.ID, element);
                model.SubdomainsDictionary[0].Elements.Add(element);
            }

            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(
                new[]
                {
                    new NodalDisplacement(model.NodesDictionary[225], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[225], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[225], StructuralDof.TranslationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[225], StructuralDof.RotationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[225], StructuralDof.RotationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[225], StructuralDof.RotationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[5], StructuralDof.TranslationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[235], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[235], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[235], StructuralDof.TranslationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[235], StructuralDof.RotationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[235], StructuralDof.RotationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[235], StructuralDof.RotationZ, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[15], StructuralDof.TranslationX, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[15], StructuralDof.TranslationY, amount: 0d),
                    new NodalDisplacement(model.NodesDictionary[15], StructuralDof.TranslationZ, amount: 0d),
                },
                Array.Empty<NodalLoad>()
            ));

            var accelerationsX = nodes.Where(x => x.ID % 10 == 0).Select(x => new NodalAcceleration(x, StructuralDof.TranslationX, 1d)).ToArray();
            var accelerationsY = nodes.Where(x => x.ID % 10 == 0).Select(x => new NodalAcceleration(x, StructuralDof.TranslationY, 1d)).ToArray();
            var accelerationsZ = nodes.Where(x => x.ID % 10 == 0 && x.ID != 10 && x.ID != 20 && x.ID != 230 && x.ID != 240).Select(x => new NodalAcceleration(x, StructuralDof.TranslationZ, 1d)).ToArray();
            var accelerationFL = new NodalAcceleration(model.NodesDictionary[10], StructuralDof.TranslationZ, 1d);
            var accelerationFR = new NodalAcceleration(model.NodesDictionary[20], StructuralDof.TranslationZ, 1d);
            var accelerationRL = new NodalAcceleration(model.NodesDictionary[230], StructuralDof.TranslationZ, 1d);
            var accelerationRR = new NodalAcceleration(model.NodesDictionary[240], StructuralDof.TranslationZ, 1d);

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
