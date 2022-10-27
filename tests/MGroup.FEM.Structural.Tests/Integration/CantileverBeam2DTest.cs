using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class CantileverBeam2DTest
	{
		[Fact]
		public void RunTest()
		{
			var model = CantileverBeam2DExample.CreateModel();
			var computedDisplacement = SolveModel(model);
			Assert.Equal(expected: CantileverBeam2DExample.expected_solution_node2_TranslationY, computedDisplacement);
		}

		private static double SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			ISolver solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel, solver);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			var computedDisplacement = algebraicModel.ExtractSingleValue(solver.LinearSystem.Solution, model.NodesDictionary[2], StructuralDof.TranslationY);

			return computedDisplacement;
		}

		
	}
}
