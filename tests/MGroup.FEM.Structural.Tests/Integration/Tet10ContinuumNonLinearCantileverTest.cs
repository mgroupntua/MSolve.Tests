using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.Direct;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.FEM.Structural.Tests.Commons;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public static class Tet10ContinuumNonLinearCantileverTest
	{

		[Fact]
		private static void RunTest()
		{
			var model = Tet10ContinuumNonLinearCantileverExample.CreateModel();
			var computedDisplacements = SolveModel(model);
			Assert.True(Utilities.AreDisplacementsSame(Tet10ContinuumNonLinearCantileverExample.GetExpectedDisplacements(), computedDisplacements, tolerance: 1E-3));
		}

		private static double[] SolveModel(Model model)
		{
			var solverFactory = new LdlSkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			ISolver solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: 2)
			{
				ResidualTolerance = 1E-8,
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1
			};

			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, loadControlAnalyzer);

			var watchDofs = new List<(INode node, IDofType dof)>()
			{
				(model.NodesDictionary[6], StructuralDof.TranslationX),
				(model.NodesDictionary[8], StructuralDof.TranslationX)
			};

			var log1 = new IncrementalDisplacementsLog(watchDofs, algebraicModel);
			loadControlAnalyzer.IncrementalDisplacementsLog = log1;

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			var solutionOfIters5And12 = new double[]
			{
				log1.GetTotalDisplacement(5, watchDofs[0].node, watchDofs[0].dof),
				log1.GetTotalDisplacement(12, watchDofs[0].node, watchDofs[0].dof),
				log1.GetTotalDisplacement(5, watchDofs[1].node, watchDofs[1].dof),
				log1.GetTotalDisplacement(12, watchDofs[1].node, watchDofs[1].dof)
			};

			return solutionOfIters5And12;
		}



	}

}
