using MGroup.MSolve.Discretization.Entities;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Constitutive.Thermal;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using MGroup.FEM.Thermal.Tests.ExampleModels;
using Xunit;

namespace MGroup.FEM.Thermal.Tests.Integration
{
	public class ThermalBenchmarkProvatidis_11_2
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
			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, linearAnalyzer);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return solver.LinearSystem.Solution.SingleVector;
		}
	}
}
