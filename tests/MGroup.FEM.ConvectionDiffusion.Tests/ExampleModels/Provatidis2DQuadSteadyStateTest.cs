using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using System.Collections.Generic;
using System;

namespace ConvectionDiffusionTest
{
    public static class Provatidis2DQuadSteadyStateTest
    {
        //                                              T5
        private static double[] prescribedSolution = { 150d };
        public static Model CreateModel()
        {
            var model = new Model();
            model.SubdomainsDictionary.Add(0, new Subdomain(0));
            var nodes = new Node[]
            {
                new Node(id : 1, x : 0, y : 0),
                new Node(id : 2, x : 1, y : 0),
                new Node(id : 3, x : 2, y : 0),
                new Node(id : 4, x : 0, y : 1),
                new Node(id : 5, x : 1, y : 1),
                new Node(id : 6, x : 2, y : 1),
                new Node(id : 7, x : 0, y : 2),
                new Node(id : 8, x : 1, y : 2),
                new Node(id : 9, x : 2, y : 2),
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
                new List<Node>() { nodes[0], nodes[1], nodes[4], nodes[3] },
                new List<Node>() { nodes[1], nodes[2], nodes[5], nodes[4] },
                new List<Node>() { nodes[3], nodes[4], nodes[7], nodes[6] },
                new List<Node>() { nodes[4], nodes[5], nodes[8], nodes[7] }

            };

            var elements = new ConvectionDiffusionElement2D[]
            {
                elementFactory.CreateElement(CellType.Quad4, elementNodes[0]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[1]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[2]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[3]),
            };

            model.ElementsDictionary.Add(0, elements[0]);
            model.SubdomainsDictionary[0].Elements.Add(elements[0]);
            model.ElementsDictionary.Add(1, elements[1]);
            model.SubdomainsDictionary[0].Elements.Add(elements[1]);
            model.ElementsDictionary.Add(2, elements[2]);
            model.SubdomainsDictionary[0].Elements.Add(elements[2]);
            model.ElementsDictionary.Add(3, elements[3]);
            model.SubdomainsDictionary[0].Elements.Add(elements[3]);


            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new[]
                {
          
                    new NodalUnknownVariable(nodes[0], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[3], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[6], ConvectionDiffusionDof.UnknownVariable, 100d)
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[]
                {
                    new NodalUnknownVariableFlux(model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable, 25d),
                    new NodalUnknownVariableFlux(model.NodesDictionary[5], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariableFlux(model.NodesDictionary[8], ConvectionDiffusionDof.UnknownVariable, 25d),
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
                if (Math.Abs((prescribedSolution[i] - numericalSolution[i]) / prescribedSolution[i]) > 1E-6)
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
