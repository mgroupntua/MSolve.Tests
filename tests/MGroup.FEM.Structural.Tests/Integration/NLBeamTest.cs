//using MGroup.MSolve.Discretization.Entities;
//using MGroup.Constitutive.Structural;
//using MGroup.NumericalAnalyzers;
//using MGroup.NumericalAnalyzers.Discretization.NonLinear;
//using MGroup.Solvers.Direct;
//using MGroup.FEM.Structural.Tests.ExampleModels;
//using Xunit;

//namespace MGroup.FEM.Structural.Tests.Integration
//{
//	public class NLBeamTest 
//	{
//		[Fact]
//		private static void RunTest()
//		{
//			var model = NLBeamExample.CreateModel();
//			SolveModel(model);
//			Assert.True(false);// TODO: This test has no prescribed results to be compare our solution against
//		}

//		private static void SolveModel(Model model)
//		{
//			var solverFactory = new SkylineSolver.Factory();
//			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
//			var solver = solverFactory.BuildSolver(algebraicModel);
//			var problem = new ProblemStructural(model, algebraicModel, solver);

//			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, problem, numIncrements: 10);
//			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
//			var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, solver, problem, loadControlAnalyzer);

//			staticAnalyzer.Initialize();
//			staticAnalyzer.Solve();
//		}
//	}
//}
