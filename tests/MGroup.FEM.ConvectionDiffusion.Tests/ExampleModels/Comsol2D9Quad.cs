using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using System.Collections.Generic;
using System;

namespace ConvectionDiffusionTest
{
    public static class Comsol2D9Quad
    {
        public static Model CreateModel(double capacityCoeff, double diffusionCoeff, double[] convectionCoeff, double dependentSourceCoeff, double independentSourceCoeff)
        {
            var model = new Model();
            model.SubdomainsDictionary.Add(0, new Subdomain(0));
            var nodes = new Node[]
            {
                new Node(id : 1, x : 0d, y : 0d),
                new Node(id : 2, x : 1d, y : 0d),
                new Node(id : 3, x : 2d, y : 0d),
                new Node(id : 4, x : 3d, y : 0d),
                new Node(id : 5, x : 0d, y : 1d),
                new Node(id : 6, x : 1d, y : 1d),
                new Node(id : 7, x : 2d, y : 1d),
                new Node(id : 8, x : 3d, y : 1d),
                new Node(id : 9, x : 0d, y : 2d),
                new Node(id : 10, x : 1d, y : 2d),
                new Node(id : 11, x : 2d, y : 2d),
                new Node(id : 12, x : 3d, y : 2d),
                new Node(id : 13, x : 0d, y : 3d),
                new Node(id : 14, x : 1d, y : 3d),
                new Node(id : 15, x : 2d, y : 3d),
                new Node(id : 16, x : 3d, y : 3d),
            };
            foreach (var node in nodes)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

            var material = new ConvectionDiffusionProperties(capacityCoeff: capacityCoeff, diffusionCoeff: diffusionCoeff, convectionCoeff: convectionCoeff , dependentSourceCoeff: dependentSourceCoeff, independentSourceCoeff: independentSourceCoeff);

            var elementFactory = new ConvectionDiffusionElement2DFactory(commonThickness: 1d, material);

            var elementNodes = new IReadOnlyList<Node>[]
            {
                new List<Node>() { nodes[0], nodes[1], nodes[5], nodes[4] },
                new List<Node>() { nodes[1], nodes[2], nodes[6], nodes[5] },
                new List<Node>() { nodes[2], nodes[3], nodes[7], nodes[6] },
                new List<Node>() { nodes[4], nodes[5], nodes[9], nodes[8] },
                new List<Node>() { nodes[5], nodes[6], nodes[10], nodes[9] },
                new List<Node>() { nodes[6], nodes[7], nodes[11], nodes[10]},
                new List<Node>() { nodes[8], nodes[9], nodes[13], nodes[12]},
                new List<Node>() { nodes[9], nodes[10], nodes[14], nodes[13]},
                new List<Node>() { nodes[10], nodes[11], nodes[15], nodes[14]}

            };

            var elements = new ConvectionDiffusionElement2D[]
            {
                elementFactory.CreateElement(CellType.Quad4, elementNodes[0]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[1]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[2]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[3]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[4]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[5]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[6]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[7]),
                elementFactory.CreateElement(CellType.Quad4, elementNodes[8])
            };

            for (int i = 0; i < elements.Length; i++)
            {
                model.ElementsDictionary.Add(i, elements[i]);
                model.SubdomainsDictionary[0].Elements.Add(elements[i]);
            }

            model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
                new[]
                {          
                    new NodalUnknownVariable(nodes[0], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[4], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[8], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[12], ConvectionDiffusionDof.UnknownVariable, 100d),
                    new NodalUnknownVariable(nodes[3], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(nodes[7], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(nodes[11], ConvectionDiffusionDof.UnknownVariable, 50d),
                    new NodalUnknownVariable(nodes[15], ConvectionDiffusionDof.UnknownVariable, 50d)
                },
                new INodalConvectionDiffusionNeumannBoundaryCondition[]{}
            ));

            return model;
        }
    }
}
