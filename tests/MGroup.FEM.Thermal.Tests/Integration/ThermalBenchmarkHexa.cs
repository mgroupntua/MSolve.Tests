using Xunit;
using System.Linq;

using MGroup.FEM.Helpers;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Thermal;
using MGroup.Constitutive.Thermal.BoundaryConditions;
using MGroup.FEM.Thermal.Isoparametric;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using MGroup.LinearAlgebra.Vectors;

namespace MGroup.FEM.Thermal.Tests.Integration
{
	public class ThermalBenchmarkHexa
	{
		[Fact]
		private static void RunTest()
		{
			Assert.True(CompareResults(SolveModel(CreateModel())));
		}

		private static Model CreateModel()
		{
			var model = new Model();
			model.SubdomainsDictionary[0] = new Subdomain(id: 0);

			int nodeIndex = -1;
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 0.0);

			var elementNodes = new INode[][]
			{
				new[] { model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12], model.NodesDictionary[10], model.NodesDictionary[1], model.NodesDictionary[0], model.NodesDictionary[9] },
				new[] { model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13], model.NodesDictionary[11], model.NodesDictionary[2], model.NodesDictionary[1], model.NodesDictionary[10] },
				new[] { model.NodesDictionary[16], model.NodesDictionary[7], model.NodesDictionary[6], model.NodesDictionary[15], model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12] },
				new[] { model.NodesDictionary[17], model.NodesDictionary[8], model.NodesDictionary[7], model.NodesDictionary[16], model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13] },
				new[] { model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21], model.NodesDictionary[19], model.NodesDictionary[10], model.NodesDictionary[9], model.NodesDictionary[18] },
				new[] { model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22], model.NodesDictionary[20], model.NodesDictionary[11], model.NodesDictionary[10], model.NodesDictionary[19] },
				new[] { model.NodesDictionary[25], model.NodesDictionary[16], model.NodesDictionary[15], model.NodesDictionary[24], model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21] },
				new[] { model.NodesDictionary[26], model.NodesDictionary[17], model.NodesDictionary[16], model.NodesDictionary[25], model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22] },
			};
			var nodeReordering = new GMeshElementLocalNodeOrdering();
			var rearrangeNodes = elementNodes.Select(x => nodeReordering.ReorderNodes(x, CellType.Hexa8)).ToArray();

			var elementFactory = new ThermalElement3DFactory(new ThermalProperties(density: 1d, specialHeatCoeff: 1d, thermalConductivity: 1d));
			for (int i = 0; i < elementNodes.Length; i++)
			{
				model.ElementsDictionary[i] = elementFactory.CreateElement(CellType.Hexa8, rearrangeNodes[i]);
				model.ElementsDictionary[i].ID = i;
				model.SubdomainsDictionary[0].Elements.Add(model.ElementsDictionary[i]);
			}
;
			model.BoundaryConditions.Add(new ThermalBoundaryConditionSet(
				new[]
				{
					new NodalTemperature(model.NodesDictionary[0], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[1], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[2], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[9], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[10], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[11], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[18], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[19], ThermalDof.Temperature, 100d),
					new NodalTemperature(model.NodesDictionary[20], ThermalDof.Temperature, 100d)
				},
				new[]
				{
					new NodalHeatFlux(model.NodesDictionary[6], ThermalDof.Temperature, 100d),
					new NodalHeatFlux(model.NodesDictionary[24], ThermalDof.Temperature, 100d),
				}
			));

			return model;
		}

		private static IVectorView SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);

			var physicsProblem = new ProblemThermal(model, algebraicModel, solver);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, physicsProblem);
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, physicsProblem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return solver.LinearSystem.Solution.SingleVector;
		}

		private static bool CompareResults(IVectorView solution)
		{
			const int numFreeDofs = 18;
			if (solution.Length != numFreeDofs)
			{
				return false;
			}
			// dofs:														4,       5,       6,       7,       8,       9,      13,      14,      15,      16,      17,      18,      22,      23,      24,      25,      26,  27
			var expectedSolution = Vector.CreateFromArray(new double[] { 135.054, 158.824, 135.054, 469.004, 147.059, 159.327, 178.178, 147.299, 139.469, 147.059, 191.717, 147.059, 135.054, 158.824, 135.054, 469.004, 147.059, 159.327 });
			var comparer = new ValueComparer(1E-3);
			for (int i = 0; i < numFreeDofs; ++i)
			{
				if (comparer.AreEqual(expectedSolution[i], solution[i]) == false)
				{
					return false;
				}
			}

			return true;
		}

	}
}
