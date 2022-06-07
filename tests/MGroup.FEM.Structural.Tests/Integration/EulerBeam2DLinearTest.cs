using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using MGroup.LinearAlgebra.Vectors;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class EulerBeam2DLinearTest
	{
		[Fact]
		public void RunTest()
		{
			var model = EulerBeam2DExample.CreateModel();
			var solution = SolveModel(model);
			Assert.Equal(expected: EulerBeam2DExample.expected_solution4, solution[4], precision: 12);
		}

		private static Vector SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel, solver);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return solver.LinearSystem.Solution.SingleVector;
		}
	}
}
