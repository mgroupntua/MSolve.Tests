//using MGroup.MSolve.Discretization.Entities;
//using MGroup.Constitutive.Structural;
//using MGroup.NumericalAnalyzers.Dynamic;
//using MGroup.Solvers.Direct;
//using MGroup.NumericalAnalyzers.Discretization.NonLinear;
//using MGroup.LinearAlgebra.Vectors;
//using MGroup.FEM.Structural.Tests.ExampleModels;
//using Xunit;

//namespace MGroup.FEM.Structural.Tests.Integration
//{

//	public class Beam3DElasticNonlinearNewmarkDynamicAnalysisTest
//	{
//		[Fact]
//		private static void RunTest()
//		{
//			var model = Beam3DElasticCorotationalQuaternionExample.CreateModel();
//			var solution = SolveModel(model);
//			Assert.Equal(expected: Beam3DElasticCorotationalQuaternionExample.expected_solution7, solution[7], precision: 12);
//		}

//		private static Vector SolveModel(Model model)
//		{
//			var solverFactory = new SkylineSolver.Factory();
//			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
//			var solver = solverFactory.BuildSolver(algebraicModel);
//			var problem = new ProblemStructural(model, algebraicModel, solver);

//			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder(model, algebraicModel, solver, problem, numIncrements: 10);
//			loadControlAnalyzerBuilder.MaxIterationsPerIncrement = 120;
//			loadControlAnalyzerBuilder.NumIterationsForMatrixRebuild = 500;

//			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();
//			var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, loadControlAnalyzer, timeStep: 0.28, totalTime: 3.36);
//			dynamicAnalyzerBuilder.SetNewmarkParametersForConstantAcceleration(); // Not necessary. This is the default
//			var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

//			dynamicAnalyzer.Initialize();
//			dynamicAnalyzer.Solve();

//			return solver.LinearSystem.RhsVector.SingleVector;
//		}
//	}
//}
