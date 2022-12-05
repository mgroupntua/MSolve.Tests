using ConvectionDiffusionTest;
using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers.Dynamic;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;

namespace MGroup.FEM.ConvectionDiffusion.Tests.Integration
{
    public class DiffStSt2DProvatidis
    {
        [Fact]
        private void RunTest()
        {
            var model = Provatidis2DQuadSteadyStateTest.CreateModel();
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var analyzer = new StaticAnalyzer(algebraicModel, problem, linearAnalyzer);
            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable),
            };
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            analyzer.Initialize();
            analyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            var numericalSolution = new double[watchDofs.Count];
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                numericalSolution[i] = log.DOFValues[watchDofs[i].node, watchDofs[i].dof];
            }
            Assert.True(Provatidis2DQuadSteadyStateTest.CheckResults(numericalSolution));
        }

    }
}
