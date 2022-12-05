using MGroup.Constitutive.ConvectionDiffusion;
using MGroup.Constitutive.ConvectionDiffusion.BoundaryConditions;
using MGroup.FEM.ConvectionDiffusion.Isoparametric;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.Discretization;
using MGroup.FEM.Helpers;
using System.Linq;
using System;

namespace ConvectionDiffusionTest
{
    public static class Comsol3D8Hexa
    {
		public static Model CreateModel(double capacityCoeff, double diffusionCoeff, double[] convectionCoeff, double dependentSourceCoeff, double independentSourceCoeff)
        {
            var model = new Model();
			model.SubdomainsDictionary[0] = new Subdomain(id: 0);

			int nodeIndex = -1;
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 2.0, y: 0.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 1.0, y: 0.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 2.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 1.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 2.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 1.0, z: 0.0);
			model.NodesDictionary[++nodeIndex] = new Node(id: nodeIndex, x: 0.0, y: 0.0, z: 0.0);

			var elementNodes = new INode[][]
			{
				new[] { model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12], model.NodesDictionary[10], model.NodesDictionary[1], model.NodesDictionary[0], model.NodesDictionary[9] },
				new[] { model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13], model.NodesDictionary[11], model.NodesDictionary[2], model.NodesDictionary[1], model.NodesDictionary[10] },
				new[] { model.NodesDictionary[16], model.NodesDictionary[7], model.NodesDictionary[6], model.NodesDictionary[15], model.NodesDictionary[13], model.NodesDictionary[4], model.NodesDictionary[3], model.NodesDictionary[12] },
				new[] { model.NodesDictionary[17], model.NodesDictionary[8], model.NodesDictionary[7], model.NodesDictionary[16], model.NodesDictionary[14], model.NodesDictionary[5], model.NodesDictionary[4], model.NodesDictionary[13] },
				new[] { model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21], model.NodesDictionary[19], model.NodesDictionary[10], model.NodesDictionary[9], model.NodesDictionary[18] },
				new[] { model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22], model.NodesDictionary[20], model.NodesDictionary[11], model.NodesDictionary[10], model.NodesDictionary[19] },
				new[] { model.NodesDictionary[25], model.NodesDictionary[16], model.NodesDictionary[15], model.NodesDictionary[24], model.NodesDictionary[22], model.NodesDictionary[13], model.NodesDictionary[12], model.NodesDictionary[21] },
				new[] { model.NodesDictionary[26], model.NodesDictionary[17], model.NodesDictionary[16], model.NodesDictionary[25], model.NodesDictionary[23], model.NodesDictionary[14], model.NodesDictionary[13], model.NodesDictionary[22] },
			};
           
            var nodeReordering = new GMeshElementLocalNodeOrdering();
			var rearrangeNodes = elementNodes.Select(x => nodeReordering.ReorderNodes(x, CellType.Hexa8)).ToArray();

            var material = new ConvectionDiffusionProperties(
				capacityCoeff: capacityCoeff,
				diffusionCoeff: diffusionCoeff,
				convectionCoeff: convectionCoeff,
				dependentSourceCoeff: dependentSourceCoeff,
				independentSourceCoeff: independentSourceCoeff);

			var elementFactory = new ConvectionDiffusionElement3DFactory(material);
            for (int i = 0; i < elementNodes.Length; i++)
			{
				model.ElementsDictionary[i] = elementFactory.CreateElement(CellType.Hexa8, rearrangeNodes[i]);
				model.ElementsDictionary[i].ID = i;
				model.SubdomainsDictionary[0].Elements.Add(model.ElementsDictionary[i]);
			}
;
			model.BoundaryConditions.Add(new ConvectionDiffusionBoundaryConditionSet(
				new[]
				{
					new NodalUnknownVariable(model.NodesDictionary[0], ConvectionDiffusionDof.UnknownVariable,  100d),
					new NodalUnknownVariable(model.NodesDictionary[1], ConvectionDiffusionDof.UnknownVariable,  100d),
					new NodalUnknownVariable(model.NodesDictionary[2], ConvectionDiffusionDof.UnknownVariable,  100d),
					new NodalUnknownVariable(model.NodesDictionary[9], ConvectionDiffusionDof.UnknownVariable,  100d),
					new NodalUnknownVariable(model.NodesDictionary[10], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[11], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[18], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[19], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[20], ConvectionDiffusionDof.UnknownVariable, 100d),
					new NodalUnknownVariable(model.NodesDictionary[6], ConvectionDiffusionDof.UnknownVariable,  50d),
					new NodalUnknownVariable(model.NodesDictionary[7], ConvectionDiffusionDof.UnknownVariable,  50d),
					new NodalUnknownVariable(model.NodesDictionary[8], ConvectionDiffusionDof.UnknownVariable,  50d),
					new NodalUnknownVariable(model.NodesDictionary[15], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[16], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[17], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[24], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[25], ConvectionDiffusionDof.UnknownVariable, 50d),
					new NodalUnknownVariable(model.NodesDictionary[26], ConvectionDiffusionDof.UnknownVariable, 50d),
				},
                new INodalConvectionDiffusionNeumannBoundaryCondition[] {} 
			));

			return model;
        }
    }
}
