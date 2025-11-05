using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class PlaneFrameTest
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

		[Fact]
		public void RunTest()
		{
			var model = PlaneFrameExample.CreateModel();
			var log = SolveModel(model);
			Assert.Equal(expected: PlaneFrameExample.expected_solution_node2_TranslationX, actual: log.DOFValues[watchDofs[0].node, watchDofs[0].dof], precision: 2);
		}

		private static DOFSLog SolveModel(Model model)
		{
			var solverFactory = new LdlSkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var loadControlAnalyserBuilder = new LoadControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: 10)
			{
				ResidualTolerance = 1E-3
			};
			var loadControlAnalyzer = loadControlAnalyserBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, loadControlAnalyzer);

			watchDofs.Add((model.NodesDictionary[2], StructuralDof.TranslationX));
			loadControlAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return (DOFSLog)loadControlAnalyzer.Logs[0];
		}
	}
}
