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
	public class CantileverBeam2DCorotationalDisplacementControlTest
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

		[Fact]
		public void RunTest()
		{
			var model = CantileverBeam2DCorotationalExample.CreateModel();
			var log = SolveModel(model);
			Assert.Equal(CantileverBeam2DCorotationalExample.expected_solution_node3_TranslationX, log.DOFValues[watchDofs[0].node, watchDofs[0].dof], precision: 5);
		}

		private static DOFSLog SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var displacementControlAnalyzerBuilder = new DisplacementControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: 10);
			var displacementControlAnalyzer = displacementControlAnalyzerBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, displacementControlAnalyzer);

			watchDofs.Add((model.NodesDictionary[3], StructuralDof.TranslationX));
			displacementControlAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return (DOFSLog)displacementControlAnalyzer.Logs[0];
		}
	}
}
