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
			IVectorView solution = SolveModel(model);
			Assert.True(Provatidis_11_2_Example.CompareResults(solution));
		}

		private static IVectorView SolveModel(Model model)
		{
			var solverFactory = new SkylineSolver.Factory();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemThermal(model, algebraicModel, solver);

			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.5, totalTime: 1000);
			dynamicAnalyzerBuilder.SetNewmarkParameters(beta: 0.5, gamma: 0.5);
			var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

			dynamicAnalyzer.Initialize();
			dynamicAnalyzer.Solve();

			return solver.LinearSystem.Solution.SingleVector;
		}
	}
}
