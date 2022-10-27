using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MGroup.FEM.Structural.Tests.Integration;

namespace MGroup.FEM.Structural.Tests
{
    public class RunTests
    {
        static void Main(string[] args)
        {
            CentralDifferencesTest.RunTest();
            Console.ReadLine();//
        }
    }
}
