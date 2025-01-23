using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Solvers.Direct;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.FEM.Structural.Tests.Commons;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{

	public static class Shell8andCohesiveNonLinearTest
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

		[Fact]
		private static void RunTest()
		{
			var model = Shell8andCohesiveNonLinearExample.CreateModel();
			var computedDisplacements = SolveModel(model);
			Assert.True(Utilities.AreDisplacementsSame(Shell8andCohesiveNonLinearExample.GetExpectedDisplacements(), computedDisplacements, tolerance: 1E-10));
		}

		private static IncrementalDisplacementsLog SolveModel(Model model)
		{
			var solverFactory = new LdlSkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: 2)
			{
				ResidualTolerance = 1E-8,
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1
			};
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, loadControlAnalyzer);

			watchDofs.Add((model.NodesDictionary[1], StructuralDof.TranslationX));
			watchDofs.Add((model.NodesDictionary[3], StructuralDof.TranslationY));
			watchDofs.Add((model.NodesDictionary[5], StructuralDof.RotationX));
			watchDofs.Add((model.NodesDictionary[8], StructuralDof.TranslationX));
			watchDofs.Add((model.NodesDictionary[8], StructuralDof.RotationY));
			var log1 = new IncrementalDisplacementsLog(watchDofs, algebraicModel);
			loadControlAnalyzer.IncrementalDisplacementsLog = log1;

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return log1;
		}
	}

}
