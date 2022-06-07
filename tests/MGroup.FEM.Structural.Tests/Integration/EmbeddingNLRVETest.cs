using System;
using System.Collections.Generic;
using System.Linq;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.FEM.Structural.Tests.Commons;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{

	public static class EmbeddingNLRVETest
	{
		[Fact]
		public static void RunTest()
		{
			var model = EmbeddingNLRVEExample.CreateModel();
			var computedDisplacements = SolveModel(model);
			Assert.True(Utilities.AreDisplacementsSame(EmbeddingNLRVEExample.GetExpectedDisplacements(), computedDisplacements, tolerance: 1E-9));
		}

		private static TotalDisplacementsPerIterationLog SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel, solver);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, problem, numIncrements: 2)
			{
				ResidualTolerance = 1E-8,
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1
			};
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, loadControlAnalyzer);

			var watchDofs = new List<(INode node, IDofType dof)>();
			watchDofs.Add((model.NodesDictionary[2], StructuralDof.TranslationX));
			watchDofs.Add((model.NodesDictionary[4], StructuralDof.TranslationY));
			watchDofs.Add((model.NodesDictionary[9], StructuralDof.TranslationX));
			watchDofs.Add((model.NodesDictionary[11], StructuralDof.TranslationZ));
			watchDofs.Add((model.NodesDictionary[15], StructuralDof.TranslationZ));
			var log1 = new TotalDisplacementsPerIterationLog(watchDofs, algebraicModel);
			loadControlAnalyzer.TotalDisplacementsPerIterationLog = log1;

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return log1;
		}
	}
}
