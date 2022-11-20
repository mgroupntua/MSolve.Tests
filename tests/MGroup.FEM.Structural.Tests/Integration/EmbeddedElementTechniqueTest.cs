using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class EmbeddedElementTechniqueTest
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

		[Fact]
		public void RunTest()
		{
			var model = EmbeddedElementExample.CreateModel();
			var log = SolveModel(model);
			Assert.Equal(expected: EmbeddedElementExample.expected_solution_node8_TranslationZ, actual: log.DOFValues[watchDofs[0].node, watchDofs[0].dof], precision: 3);
		}

		private static DOFSLog SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel, solver);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: 10);
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, loadControlAnalyzer);

			watchDofs.Add((model.NodesDictionary[8], StructuralDof.TranslationZ));
			loadControlAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return (DOFSLog)loadControlAnalyzer.Logs[0];
		}

	}
}
