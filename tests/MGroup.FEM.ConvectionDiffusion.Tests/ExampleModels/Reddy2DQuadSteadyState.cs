using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using System.Collections.Generic;
using System;

namespace ConvectionDiffusionTest
{
    /// <summary>
    /// An introduction to finite elements method  example 8.5.1 pg. 464
    /// </summary>
    public static class Reddy2dHeatDiffusionTest
    {
        //                                             T6         , T7
        private static double[] prescribedSolution = { 0.60881558d, 0.35149979 };
        public static Model CreateModel()
        {
            var model = new Model();
            model.SubdomainsDictionary.Add(0, new Subdomain(0));
            var nodes = new Node[]
            {
                new Node(id: 1,  x: 0, y: 0),
                new Node(id: 2,  x: 1, y: 0),
                new Node(id: 3,  x: 2, y: 0),
                new Node(id: 4,  x: 3, y: 0),
                new Node(id: 5,  x: 0, y: 1),
                new Node(id: 6,  x: 1, y: 1),
                new Node(id: 7,  x: 2, y: 1),
                new Node(id: 8,  x: 3, y: 1),
                new Node(id: 9,  x: 0, y: 2),
                new Node(id: 10, x: 1, y: 2),
                new Node(id: 11, x: 2, y: 2),
                new Node(id: 12, x: 3, y: 2)
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

            var material = new ConvectionDiffusionProperties(capacityCoeff: 0d, diffusionCoeff: 1d, convectionCoeff: new[] {0d, 0d} , dependentSourceCoeff: 0d, independentSourceCoeff: 0d);

            var nodesDictionary = model.NodesDictionary;
            var elementFactory = new ConvectionDiffusionElement2DFactory(commonThickness: 1, material);

            var elementNodes = new IReadOnlyList<Node>[]
            {


                new List<Node>() { nodes[0], nodes[1], nodes[5],  nodes[4]  },
                new List<Node>() { nodes[1], nodes[2], nodes[6],  nodes[5]  },
                new List<Node>() { nodes[2], nodes[3], nodes[7],  nodes[6]  },
                new List<Node>() { nodes[4], nodes[5], nodes[9],  nodes[8]  },
                new List<Node>() { nodes[5], nodes[6], nodes[10], nodes[9]  },
                new List<Node>() { nodes[6], nodes[7], nodes[11], nodes[10] }

            };

            var elements = new ConvectionDiffusionElement2D[]
            {
                elementFactory.CreateElement(CellType.Quad4, elementNodes[0]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[1]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[2]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[3]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[4]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[5]),
            };

            model.ElementsDictionary.Add(0, elements[0]);
            model.SubdomainsDictionary[0].Elements.Add(elements[0]);
            model.ElementsDictionary.Add(1, elements[1]);
            model.SubdomainsDictionary[0].Elements.Add(elements[1]);
            model.ElementsDictionary.Add(2, elements[2]);
            model.SubdomainsDictionary[0].Elements.Add(elements[2]);
            model.ElementsDictionary.Add(3, elements[3]);
            model.SubdomainsDictionary[0].Elements.Add(elements[3]);
            model.ElementsDictionary.Add(4, elements[4]);
            model.SubdomainsDictionary[0].Elements.Add(elements[4]);
            model.ElementsDictionary.Add(5, elements[5]);
            model.SubdomainsDictionary[0].Elements.Add(elements[5]);

            var T0 = 1;
            var pi = Math.Acos(-1d);
            var a = 1d;
            Func<double, double> boundaryTemperature = (x) => T0 * Math.Cos((pi * x) / (6d * a));
            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new[]
                {
                    new NodalUnknownVariable(nodes[3], ConvectionDiffusionDof.UnknownVariable,  0d),
                    new NodalUnknownVariable(nodes[7], ConvectionDiffusionDof.UnknownVariable,  0d),
                    new NodalUnknownVariable(nodes[11], ConvectionDiffusionDof.UnknownVariable, boundaryTemperature(nodes[11].X)),
                    new NodalUnknownVariable(nodes[10], ConvectionDiffusionDof.UnknownVariable, boundaryTemperature(nodes[10].X)),
                    new NodalUnknownVariable(nodes[9], ConvectionDiffusionDof.UnknownVariable,  boundaryTemperature(nodes[9].X)),
                    new NodalUnknownVariable(nodes[8], ConvectionDiffusionDof.UnknownVariable,  boundaryTemperature(nodes[8].X))
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[]
                {
                    new NodalUnknownVariableFlux(nodes[0], ConvectionDiffusionDof.UnknownVariable, 0d),
                    new NodalUnknownVariableFlux(nodes[1], ConvectionDiffusionDof.UnknownVariable, 0d),
                    new NodalUnknownVariableFlux(nodes[2], ConvectionDiffusionDof.UnknownVariable, 0d),
                    new NodalUnknownVariableFlux(nodes[3], ConvectionDiffusionDof.UnknownVariable, 0d),
                    new NodalUnknownVariableFlux(nodes[4], ConvectionDiffusionDof.UnknownVariable, 0d),
                    new NodalUnknownVariableFlux(nodes[8], ConvectionDiffusionDof.UnknownVariable, 0d),
                }
            ));

            return model;
        }

        public static bool CheckResults(double[] numericalSolution)
        {
            if (numericalSolution.Length != prescribedSolution.Length)
            {
                Console.WriteLine("Array Lengths do not match");
                return false;
            }

            var isAMatch = true;
            for (int i = 0; i < numericalSolution.Length; i++)
            {
                if (Math.Abs((prescribedSolution[i] - numericalSolution[i])/ prescribedSolution[i]) > 1E-6)
                {
                    isAMatch = false;
                    break;
                }
            }
            if (isAMatch == true)
            {
                Console.WriteLine("MSolve Solution matches prescribed solution");
                Console.WriteLine("Test Passed!");
            }
            else
            {
                Console.WriteLine("MSolve Solution does not match prescribed solution");
                Console.WriteLine("Test Failed!");
            }
            return isAMatch;
        }
    }
}
