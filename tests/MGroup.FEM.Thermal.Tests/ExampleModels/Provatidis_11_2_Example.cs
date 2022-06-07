using System;
using MGroup.MSolve.Discretization.Entities;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.DataStructures;
using MGroup.Constitutive.Thermal;
using MGroup.FEM.Thermal.Isoparametric;
using MGroup.Constitutive.Thermal.BoundaryConditions;

namespace MGroup.FEM.Thermal.Tests.ExampleModels
{
	public static class Provatidis_11_2_Example
	{
		//																		dofs:   1,   2,   4,   5,   7,   8
		private static Vector expectedSolution = Vector.CreateFromArray(new double[] { 150, 200, 150, 200, 150, 200 });
		private const int numFreeDofs = 6;

		public static Model CreateModel()
		{
			var model = new Model();

			model.SubdomainsDictionary.Add(key: 0, new Subdomain(id: 0));

			var nodes = new Node[]
			{
				new Node(id: 0, x: 0.0, y: 0.0),
				new Node(id: 1, x: 1.0, y: 0.0),
				new Node(id: 2, x: 2.0, y: 0.0),
				new Node(id: 3, x: 0.0, y: 1.0),
				new Node(id: 4, x: 1.0, y: 1.0),
				new Node(id: 5, x: 2.0, y: 1.0),
				new Node(id: 6, x: 0.0, y: 2.0),
				new Node(id: 7, x: 1.0, y: 2.0),
				new Node(id: 8, x: 2.0, y: 2.0)
			};

			foreach (var node in nodes)
			{
				model.NodesDictionary.Add(node.ID, node);
			}

			var elementFactory = new ThermalElement2DFactory(commonThickness: 1.0, new ThermalProperties(density: 1d, specialHeatCoeff: 1d, thermalConductivity: 1d));
			var elements = new ThermalElement2D[]
			{
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[0], nodes[1], nodes[4], nodes[3] }),
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[1], nodes[2], nodes[5], nodes[4] }),
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[3], nodes[4], nodes[7], nodes[6] }),
				elementFactory.CreateElement(CellType.Quad4, new Node[] { nodes[4], nodes[5], nodes[8], nodes[7] })
			};

			for (int i = 0; i < elements.Length; ++i)
			{
				elements[i].ID = i;
				model.ElementsDictionary[i] = elements[i];
				model.SubdomainsDictionary[0].Elements.Add(elements[i]);
			}

			model.BoundaryConditions.Add(new ThermalBoundaryConditionSet(
				new[]
				{
					new NodalTemperature(model.NodesDictionary[0], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[3], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[6], ThermalDof.Temperature, 100d),
				},
				new[]
				{
					new NodalHeatFlux(model.NodesDictionary[2], ThermalDof.Temperature, 25d),
					new NodalHeatFlux(model.NodesDictionary[5], ThermalDof.Temperature, 50d),
					new NodalHeatFlux(model.NodesDictionary[8], ThermalDof.Temperature, 25d),
				}
			));

			return model;
		}

		public static bool CompareResults(IVectorView solution)
		{
			var comparer = new ValueComparer(1E-8);
			if (solution.Length != numFreeDofs) return false;
			for (int i = 0; i < numFreeDofs; ++i)
			{
				if (!comparer.AreEqual(expectedSolution[i], solution[i])) return false;
			}
			return true;
		}
	}
}
