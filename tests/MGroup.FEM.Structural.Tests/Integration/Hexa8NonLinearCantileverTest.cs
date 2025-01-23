using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization.Dofs;
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

	public static class Hexa8NonLinearCantileverTest
	{
		[Fact]
		private static void RunTest()
		{
			var model = Hexa8NonLinearCantileverExample.CreateModel();
			var computedDisplacements = SolveModel(model);
			Assert.True(Utilities.AreDisplacementsSame(Hexa8NonLinearCantileverExample.GetExpectedDisplacements(), computedDisplacements, tolerance: 1E-13));
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
			
			loadControlAnalyzer.IncrementalDisplacementsLog = new IncrementalDisplacementsLog(
				new List<(INode node, IDofType dof)>()
				{
					(model.NodesDictionary[5], StructuralDof.TranslationX),
					(model.NodesDictionary[8], StructuralDof.TranslationZ),
					(model.NodesDictionary[12], StructuralDof.TranslationZ),
					(model.NodesDictionary[16], StructuralDof.TranslationZ),
					(model.NodesDictionary[20], StructuralDof.TranslationZ)
				}, algebraicModel
			);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return loadControlAnalyzer.IncrementalDisplacementsLog;
		}
	}
}
