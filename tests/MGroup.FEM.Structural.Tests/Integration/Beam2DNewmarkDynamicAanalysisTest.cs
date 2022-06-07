//using MGroup.NumericalAnalyzers.Dynamic;
//using MGroup.MSolve.Discretization.Entities;
//using MGroup.Solvers.Direct;
//using Xunit;
//using MGroup.Constitutive.Structural;
//using MGroup.NumericalAnalyzers;
//using MGroup.LinearAlgebra.Vectors;
//using MGroup.FEM.Structural.Tests.ExampleModels;

//namespace MGroup.FEM.Structural.Tests.Integration
//{
//    public class Beam2DNewmarkDynamicAanalysisTest
//    {
//		[Fact]
//		public void RunTest()
//		{
//			var model = Beam2DExample.CreateModel();
//			var solution = SolveModel(model);
//			Assert.Equal(expected: Beam2DExample.expected_solution1, actual: solution[1], precision: 8);
//		}

//		private static Vector SolveModel(Model model)
//		{
//			var solverFactory = new SkylineSolver.Factory();
//			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
//			var solver = solverFactory.BuildSolver(algebraicModel);
//			var problem = new ProblemStructural(model, algebraicModel, solver);

//			var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
//			var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder(model, algebraicModel, solver, problem, linearAnalyzer, timeStep: 0.28, totalTime: 3.36);
//			dynamicAnalyzerBuilder.SetNewmarkParametersForConstantAcceleration();
//			var dynamicAnalyzer = dynamicAnalyzerBuilder.Build();

//			dynamicAnalyzer.Initialize();
//			dynamicAnalyzer.Solve();

//			return solver.LinearSystem.RhsVector.SingleVector;
//		}
//	}
//}
