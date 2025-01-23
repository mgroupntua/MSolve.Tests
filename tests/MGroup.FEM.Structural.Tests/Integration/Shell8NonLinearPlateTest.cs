using System.Collections.Generic;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Numerics.Integration.Quadratures;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.FEM.Structural.Shells;
using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.Shells;
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Direct;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;
using MGroup.FEM.Structural.Tests.Commons;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public static class Shell8NonLinearPlateTest
	{
		[Fact]
		private static void RunTest()
		{
			var model = Shell8NonLinearPlateExample.CreateModel();
			var computedDisplacements = SolveModel(model);
			Assert.True(Utilities.AreDisplacementsSame(Shell8NonLinearPlateExample.GetExpectedDisplacements(), computedDisplacements, tolerance: 1E-11));
		}

		private static IncrementalDisplacementsLog SolveModel(Model model)
		{
			var solverFactory = new LdlSkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(algebraicModel, solver, problem, numIncrements: 2);
			loadControlAnalyzerBuilder.ResidualTolerance = 1E-8;
			loadControlAnalyzerBuilder.MaxIterationsPerIncrement = 100;
			loadControlAnalyzerBuilder.NumIterationsForMatrixRebuild = 1;
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, loadControlAnalyzer);

			var watchDofs = new List<(INode node, IDofType dof)>
			{
				(model.NodesDictionary[7], StructuralDof.TranslationX),
				(model.NodesDictionary[9], StructuralDof.TranslationY),
				(model.NodesDictionary[13], StructuralDof.RotationX),
				(model.NodesDictionary[18], StructuralDof.TranslationX),
				(model.NodesDictionary[30], StructuralDof.TranslationZ)
			};
			var log1 = new IncrementalDisplacementsLog(watchDofs, algebraicModel);
			loadControlAnalyzer.IncrementalDisplacementsLog = log1;

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return log1;
		}
	}
}
