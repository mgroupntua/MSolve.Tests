using ConvectionDiffusionTest;
using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.LinearAlgebra.Vectors;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers;
using MGroup.Solvers.Direct;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Xunit;
using MGroup.FEM.ConvectionDiffusion.Tests.Commons;

namespace MGroup.FEM.ConvectionDiffusion.Tests.Integration
{
    public class ConvDiffProdStSt8HexaComSol
    {
        [Fact]
        private void RunTest()
        {
            double[] convectionCoeff = new[] { 1d, 1d, 1d };
            double diffusionCoeff = 1d;
            double capacityCoeff = 0d;
            double dependentSourceCoeff = 1d;
            double independentSourceCoeff = 1d;

            double[] prescribedSolution = new double[] { 113.24999999999996 }; // [1, 1, 1] node id 13
            double tolerance = 1E-6;

            var model = Comsol3DComsolMesh.CreateModelFromComsolFile("../../../DataFiles/3d8Hexa.mphtxt", capacityCoeff, diffusionCoeff, convectionCoeff, dependentSourceCoeff, independentSourceCoeff);
            var solverFactory = new DenseMatrixSolver.Factory() { IsMatrixPositiveDefinite = false};
            var algebraicModel = solverFactory.BuildAlgebraicModel(model);
            var solver = solverFactory.BuildSolver(algebraicModel);
            var problem = new ProblemConvectionDiffusion(model, algebraicModel);

            var linearAnalyzer = new LinearAnalyzer(algebraicModel, solver, problem);

            var analyzer = new StaticAnalyzer(algebraicModel, problem, linearAnalyzer);

            var watchDofs = new List<(INode node, IDofType dof)>()
            {
                (model.NodesDictionary[13], ConvectionDiffusionDof.UnknownVariable),
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
            Assert.True(ResultChecker.CheckResults(numericalSolution, prescribedSolution, tolerance));
        }
    }
}
