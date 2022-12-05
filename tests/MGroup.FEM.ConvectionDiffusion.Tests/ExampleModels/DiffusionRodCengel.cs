using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Line;
using MGroup.MSolve.Discretization.Entities;
using System;

namespace ConvectionDiffusionTest
{
    public static class DiffusionRodCengel
    {
        public static Model CreateModel()
        {
            var model = new Model();
            model.SubdomainsDictionary.Add(0, new Subdomain(0));
            var nodes = new Node[]
            {
                new Node(id : 0, x : 0d,   y : 0d),
                new Node(id : 1, x : 1E-1, y : 0d),
                new Node(id : 2, x : 2E-1, y : 0d),
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }
            
            var material = new ConvectionDiffusionProperties(capacityCoeff : 0d, diffusionCoeff : 1d, convectionCoeff : new[] {0d} , dependentSourceCoeff : 0d, independentSourceCoeff : 0d);

            var elements = new ConvectionDiffusionRod[]
            {
                new ConvectionDiffusionRod(new [] {nodes[0], nodes[1]}, 0.1, material),
                new ConvectionDiffusionRod(new [] {nodes[1], nodes[2]}, 0.1, material)
            };

            model.ElementsDictionary.Add(0, elements[0]);
            model.SubdomainsDictionary[0].Elements.Add(elements[0]);

            model.ElementsDictionary.Add(1, elements[1]);
            model.SubdomainsDictionary[0].Elements.Add(elements[1]);

            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new []
                {
                    new NodalUnknownVariable(nodes[0], ConvectionDiffusionDof.UnknownVariable, 120d),
                    new NodalUnknownVariable(nodes[2], ConvectionDiffusionDof.UnknownVariable, 50d)     
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[] {} 
            ));
            return model;
        }

        public static Func<double, double> rodAnalyticalSolution = (x) => -350d * x + 120d;
        //public static Func<double, double> rodAnalyticalSolution = (x) => 200d / 2E-1 * x;

        public static bool CheckResults (double numericalSolution)
        {
            var analyticalSolution = rodAnalyticalSolution(0.1);
            
            Console.WriteLine("Analytical Solution = " + analyticalSolution);
            Console.WriteLine("Numerical Solution = " + numericalSolution);

            if ( Math.Abs((analyticalSolution - numericalSolution) / analyticalSolution ) <= 1E-6)
            {
                Console.WriteLine("MSolve solution matches analytical solution.");
                Console.WriteLine("Test Passed!");
                return true;
            }
            else
            {
                Console.WriteLine("MSolve solution does not match analytical solution.");
                Console.WriteLine("Test Failed!");
                return false;
            }
        }
    }
}
