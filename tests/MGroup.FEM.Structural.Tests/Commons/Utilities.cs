using System.Collections.Generic;
using MGroup.MSolve.DataStructures;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.MSolve.Discretization.Entities;
using Xunit;

namespace MGroup.FEM.Structural.Tests.Commons
{
	public static class Utilities
	{
		public static bool AreDisplacementsSame(IReadOnlyList<double[]> expectedDisplacements,
			TotalDisplacementsPerIterationLog computedDisplacements, double tolerance)
		{
			var comparer = new ValueComparer(tolerance);
			for (var iter = 0; iter < expectedDisplacements.Count; ++iter)
			{
				for (var i = 0; i < expectedDisplacements[iter].Length; ++i)
				{
					var expected = expectedDisplacements[iter][i];
					(var node, var dof) = computedDisplacements.WatchDofs[i];
					var computed = computedDisplacements.GetTotalDisplacement(iter, node, dof);

					if (!comparer.AreEqual(expected, computed)) return false;
				}
			}
			return true;
		}

		public static bool AreDisplacementsSame(IReadOnlyList<double[]> expectedDisplacements, IncrementalDisplacementsLog computedDisplacements, double tolerance)
		{
			var comparer = new ValueComparer(tolerance);
			for (var iter = 0; iter < expectedDisplacements.Count; ++iter)
			{
				for (var i = 0; i < expectedDisplacements[iter].Length; ++i)
				{
					var expected = expectedDisplacements[iter][i];
					(var node, var dof) = computedDisplacements.WatchDofs[i];
					var computed = computedDisplacements.GetTotalDisplacement(iter, node, dof);

					if (!comparer.AreEqual(expected, computed)) return false;
				}
			}
			return true;
		}

		public static bool AreDisplacementsSame(double[] expectedDisplacements, double[] computedDisplacements, double tolerance)
		{
			var comparer = new ValueComparer(tolerance);

			for (var i = 0; i < expectedDisplacements.Length; i++)
			{
				if (!comparer.AreEqual(expectedDisplacements[i], computedDisplacements[i])) return false;
			}

			return true;
		}

		public static void CheckModelSubdomains(Dictionary<int, int[]> expectedSubdomains, Model model)
		{
			for (var i = 0; i < expectedSubdomains.Count; i++)
			{
				var subdomainElements = model.SubdomainsDictionary[i].Elements;
				Assert.Equal(expectedSubdomains[i].Length, model.SubdomainsDictionary[i].Elements.Count);
				for (var j = 0; j < expectedSubdomains[i].Length; j++)
				{
					Assert.Equal(expectedSubdomains[i][j], subdomainElements[j].ID);
				}
			}
		}
	}
}
