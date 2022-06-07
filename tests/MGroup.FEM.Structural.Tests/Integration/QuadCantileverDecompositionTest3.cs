using System.Collections.Generic;

using MGroup.FEM.Structural.Tests.Commons;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.MSolve.Discretization.Entities;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public class QuadCantileverDecompositionTest3
	{
		private static readonly Dictionary<int, int[]> expectedSubdomains = new Dictionary<int, int[]>()
		{
			{ 0, new int[] {0,2,1,3}},
			{ 1, new int[] {4,6,5,7}},
			{ 2, new int[] {8,9}}
		};

		[Fact]
		private static void RunTest()
		{
			var model = QuadCantileverDecompositionExample3.CreateModel();

			model.ConnectDataStructures();
			var domainDecomposer = new AutomaticDomainDecomposer(model, 3);
			domainDecomposer.UpdateModel();

			Utilities.CheckModelSubdomains(expectedSubdomains, model);
		}
	}
}
