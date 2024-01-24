using MGroup.MSolve.Discretization.Entities;
using MGroup.LinearAlgebra.Iterative.PreconditionedConjugateGradient;
using MGroup.LinearAlgebra.Iterative.Termination;
using MGroup.MSolve.Discretization.Dofs;
using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.Solvers.Iterative;
using MGroup.FEM.Structural.Tests.ExampleModels;
using Xunit;
using MGroup.LinearAlgebra.Iterative.Termination.Iterations;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public static class Quad4LinearCantileverTest
	{
		private static List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();

        [Fact]
		private static void RunPcgTest()
		{
			var model = Quad4LinearCantileverExample.CreateModel();
			var log = SolvePcgModel(model);
			Assert.Equal(expected: Quad4LinearCantileverExample.expected_solution_node3_TranslationX, actual: log.DOFValues[watchDofs[0].node, watchDofs[0].dof], precision: 8);
		}

		private static DOFSLog SolvePcgModel(Model model)
		{
			var pcgBuilder = new PcgAlgorithm.Builder
			{
				ResidualTolerance = 1E-6,
				MaxIterationsProvider = new PercentageMaxIterationsProvider(maxIterationsOverMatrixOrder: 0.5)
			};

			var solverFactory = new PcgSolver.Factory();
			solverFactory.PcgAlgorithm = pcgBuilder.Build();
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var linearnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
			var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, linearnalyzer);

			watchDofs.Add((model.NodesDictionary[3], StructuralDof.TranslationX));

			linearnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

			staticAnalyzer.Initialize();
			staticAnalyzer.Solve();

			return (DOFSLog)linearnalyzer.Logs[0];
		}

        [Fact]
        private static void RunBlockPcgTest()
        {
            var model = Quad4LinearCantileverExample.CreateModel();
            var log = SolveBlockPcgModel(model);
            Assert.Equal(expected: Quad4LinearCantileverExample.expected_solution_node3_TranslationX, actual: log.DOFValues[watchDofs[0].node, watchDofs[0].dof], precision: 8);
        }

        private static DOFSLog SolveBlockPcgModel(Model model)
        {
            var pcgBuilder = new BlockPcgAlgorithm.Builder
            {
                ResidualTolerance = 1E-6,
                MaxIterationsProvider = new PercentageMaxIterationsProvider(maxIterationsOverMatrixOrder: 0.5)
            };

            var solverFactory = new BlockPcgSolver.Factory();
            solverFactory.BlockPcgAlgorithm = pcgBuilder.Build();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemStructural(model, algebraicModel);

            var linearnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);
            var staticAnalyzer = new StaticAnalyzer(algebraicModel, problem, linearnalyzer);

            watchDofs.Add((model.NodesDictionary[3], StructuralDof.TranslationX));

            linearnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            staticAnalyzer.Initialize();
            staticAnalyzer.Solve();

            return (DOFSLog)linearnalyzer.Logs[0];
        }
    }
}
