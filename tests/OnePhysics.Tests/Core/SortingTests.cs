using System;
using System.Collections.Generic;
using RecursiveAI.OnePhysics.Core;
using Xunit;

namespace RecursiveAI.OnePhysics.Tests.Core;

public class SortingTests
{
    private readonly struct IntOrder : IComparer<int>
    {
        public int Compare(int x, int y) => x.CompareTo(y);
    }

    // Reverse-sorted, constant, organ-pipe and random inputs, short and long, with many duplicates: the shapes that break a careless quicksort partition.
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(24)]
    [InlineData(25)]
    [InlineData(1000)]
    [InlineData(20000)]
    public void MatchesTheLibrarySort(int n)
    {
        var rng = new Random(n + 1);
        for (int kind = 0; kind < 6; kind++)
        {
            var a = new int[n];
            for (int i = 0; i < n; i++)
            {
                a[i] = kind switch
                {
                    0 => rng.Next(),
                    1 => rng.Next(5),
                    2 => n - i,
                    3 => i,
                    4 => 7,
                    _ => Math.Min(i, n - i),
                };
            }

            var expected = (int[])a.Clone();
            Array.Sort(expected);
            Sorting.Sort<int, IntOrder>(a, default);
            Assert.Equal(expected, a);
        }
    }

    [Fact]
    public void SortsWithoutAllocating()
    {
        var a = new int[5000];
        var rng = new Random(3);
        for (int i = 0; i < a.Length; i++) { a[i] = rng.Next(); }
        Sorting.Sort<int, IntOrder>(a, default); // warm up
        for (int i = 0; i < a.Length; i++) { a[i] = rng.Next(); }
        long before = GC.GetAllocatedBytesForCurrentThread();
        Sorting.Sort<int, IntOrder>(a, default);
        Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
    }
}
