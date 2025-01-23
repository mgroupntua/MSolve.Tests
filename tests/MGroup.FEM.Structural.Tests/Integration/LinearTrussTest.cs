using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural;
using MGroup.Solvers.Direct;
using MGroup.NumericalAnalyzers;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class LinearTrussTest
	{
		[Fact]
		private static void RunTest()
		{
			var model = LinearTrussExample.CreateModel();
			var solution = SolveModel(model);

			Assert.Equal(LinearTrussExample.expected_solution0, solution[0], precision: 10);
			Assert.Equal(LinearTrussExample.expected_solution1, solution[1], precision: 10);
		}

		private static LinearAlgebra.Vectors.Vector SolveModel(Model model)
		{
			var solverFactory = new LdlSkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return solver.LinearSystem.Solution.SingleVector;
		}
	}
}
