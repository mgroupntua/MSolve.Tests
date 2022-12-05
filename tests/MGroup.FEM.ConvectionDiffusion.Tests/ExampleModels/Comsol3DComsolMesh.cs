using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.FEM.Helpers;
using System;
using System.Collections.Generic;
using System.Linq;

namespace ConvectionDiffusionTest
{
    public static class Comsol3DComsolMesh
    {
		public static Model CreateModelFromComsolFile(string filename, double capacityCoeff, double diffusionCoeff, double[] convectionCoeff, double dependentSourceCoeff, double independentSourceCoeff)
		{
			var model = new Model();
			model.SubdomainsDictionary[0] = new Subdomain(id: 0);
			
			var reader = new ComsolMeshReader(filename);

            foreach (var node in reader.NodesDictionary.Values)
            {
                model.NodesDictionary.Add(node.ID, node);
            }

			var material = new ConvectionDiffusionProperties(
				capacityCoeff: capacityCoeff,
				diffusionCoeff: diffusionCoeff,
				convectionCoeff: convectionCoeff,
				dependentSourceCoeff: dependentSourceCoeff,
				independentSourceCoeff: independentSourceCoeff);

			var elementFactory = new ConvectionDiffusionElement3DFactory(material);

			foreach (var elementConnectivity in reader.ElementConnectivity)
			{
				var element = elementFactory.CreateElement(elementConnectivity.Value.Item1, elementConnectivity.Value.Item2);
				model.ElementsDictionary.Add(elementConnectivity.Key, element);
				model.SubdomainsDictionary[0].Elements.Add(element);
			}

			var topNodes = new List<INode>();
            var bottomNodes = new List<INode>();
            
            foreach (var node in model.NodesDictionary.Values)
            {
                if (Math.Abs(2 - node.Z) < 1E-9) topNodes.Add(node);
                if (Math.Abs(0 - node.Z) < 1E-9) bottomNodes.Add(node);
            }

			int i = 0;
            var dirichletBCs = new NodalUnknownVariable[topNodes.Count + bottomNodes.Count];
			foreach (var node in topNodes)
			{
				dirichletBCs[i] = new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, 100d);
				i++;
			}
			foreach (var node in bottomNodes)
			{
				dirichletBCs[i] = new NodalUnknownVariable(node, ConvectionDiffusionDof.UnknownVariable, 50d);
				i++;
			}

			model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
				dirichletBCs,
				new INodalConvectionDiffusionNeumannBoundaryCondition[] { }
			));

			return model;
		}
    }
}
