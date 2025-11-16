using MGroup.MSolve.Discretization.Entities;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Constitutive.Thermal;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.Solvers.Direct;
using MGroup.FEM.Thermal.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Thermal.Tests.Integration
{
	public class ThermalDynamicAnalysisTest
	{
		[Fact]
		private static void RunTest()
		{
			Model model = Provatidis_11_2_Example.CreateModel();
			IReadOnlyVector solution = SolveModel(model);
			Assert.True(Provatidis_11_2_Example.CompareResults(solution));
		}

		private static IReadOnlyVector SolveModel(Model model)
		{
			var solverFactory = new LdlSkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemThermal(model, algebraicModel);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(algebraicModel, problem, linearAnalyzer, timeStep: 0.5, totalTime: 1000, false);
			dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.5, gamma: 0.5);
			var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

			dynamicAnalyzer.Initialize();
			dynamicAnalyzer.Solve();

			return solver.LinearSystem.Solution;
		}
	}
}
