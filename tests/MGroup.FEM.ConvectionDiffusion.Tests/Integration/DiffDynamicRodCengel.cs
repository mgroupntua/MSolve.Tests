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
    public class DiffDynamicRodCengel
    {
        [Fact]
        private void RunTest()
        {
            var model = DiffusionRodCengel.CreateModel();
            var solverFactory = new SkylineSolver.Factory();
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var dynamicAnalyzerBuilder = new BDFDynamicAnalyzer.Builder(algebraicModel, problem, linearAnalyzer, timeStep: 0.5, totalTime: 1000, bdfOrder: 5);
            var analyzer = dynamicAnalyzerBuilder.Build();

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[1], ConvectionDiffusionDof.UnknownVariable),
            };
            linearAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);

            analyzer.Initialize();
            analyzer.Solve();

            DOFSLog log = (DOFSLog)linearAnalyzer.Logs[0];
            Assert.True(DiffusionRodCengel.CheckResults(log.DOFValues[watchDofs[0].node, watchDofs[0].dof]));
        }
    }
}
