using System.Collections.Generic;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.MSolve.Solution;
using MGroup.Solvers.Direct;
using Xunit;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.FEM.Structural.Tests.ExampleModels;

namespace MGroup.FEM.Structural.Tests.Integration
{

	public static class CentralDifferencesTest
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();
		[Fact]
		public static void RunTest()
		{
			var model = MockStructuralModel.CreateModel();
			var log = SolveModel(model);

			Assert.Equal(MockStructuralModel.expected_solution_node0_TranslationX, log.DOFValues[watchDofs[0].node, watchDofs[0].dof], precision: 8);
			Assert.Equal(MockStructuralModel.expected_solution_node0_TranslationY, log.DOFValues[watchDofs[1].node, watchDofs[1].dof], precision: 8);
		}

		private static DOFSLog SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel, solver);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var dynamicAnalyzerBuilder = new CentralDifferences.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.28, totalTime: 3.36);
			var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

			watchDofs.Add((model.NodesDictionary[0], StructuralDof.TranslationX));
			watchDofs.Add((model.NodesDictionary[0], StructuralDof.TranslationY));
			linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			dynamicAnalyzer.Initialize();
			dynamicAnalyzer.Solve();

			return (DOFSLog)linearAnalyzer.Logs[0];
		}
	}
}
