using System;
using System.Collections.Generic;

namespace RecursiveAI.OnePhysics.Core;

/// <summary>
/// Sorting by a struct comparer without allocating: <c>Span.Sort</c> and <c>Array.Sort</c> with a comparer wrap it in a new delegate on every call, which shows up as garbage in every step.
/// The orders sorted here are total (equal elements are identical), so any correct sort gives the same result and the choice of algorithm cannot change a world.
/// </summary>
internal static class Sorting
{
    /// <summary>Introsort: quicksort on a median of three, insertion sort for short runs, heap sort when the recursion gets deep.</summary>
    public static void Sort<T, TComparer>(Span<T> a, TComparer cmp) where TComparer : struct, IComparer<T> => Sort(a, cmp, 0);

    private static void Sort<T, TComparer>(Span<T> a, TComparer cmp, int depth) where TComparer : struct, IComparer<T>
    {
        while (a.Length > 24)
        {
            if (depth++ > 48) { HeapSort(a, cmp); return; }
            int mid = a.Length >> 1, last = a.Length - 1;
            if (cmp.Compare(a[mid], a[0]) < 0) { (a[mid], a[0]) = (a[0], a[mid]); }
            if (cmp.Compare(a[last], a[0]) < 0) { (a[last], a[0]) = (a[0], a[last]); }
            if (cmp.Compare(a[last], a[mid]) < 0) { (a[last], a[mid]) = (a[mid], a[last]); }
            (a[0], a[mid]) = (a[mid], a[0]);
            T pivot = a[0];
            int i = 1, j = last;
            while (true)
            {
                while (i <= j && cmp.Compare(a[i], pivot) < 0) { i++; }
                while (i <= j && cmp.Compare(pivot, a[j]) < 0) { j--; }
                if (i >= j) { break; }
                (a[i], a[j]) = (a[j], a[i]);
                i++; j--;
            }

            (a[0], a[j]) = (a[j], a[0]);
            // Recurse into the smaller side and loop on the larger, so the depth stays logarithmic.
            if (j < a.Length - 1 - j) { Sort(a.Slice(0, j), cmp, depth); a = a.Slice(j + 1); }
            else { Sort(a.Slice(j + 1), cmp, depth); a = a.Slice(0, j); }
        }

        for (int k = 1; k < a.Length; k++)
        {
            T x = a[k];
            int m = k - 1;
            while (m >= 0 && cmp.Compare(x, a[m]) < 0) { a[m + 1] = a[m]; m--; }
            a[m + 1] = x;
        }
    }

    private static void HeapSort<T, TComparer>(Span<T> a, TComparer cmp) where TComparer : struct, IComparer<T>
    {
        int n = a.Length;
        for (int s = (n >> 1) - 1; s >= 0; s--) { SiftDown(a, s, n, cmp); }
        for (int e = n - 1; e > 0; e--) { (a[0], a[e]) = (a[e], a[0]); SiftDown(a, 0, e, cmp); }
    }

    private static void SiftDown<T, TComparer>(Span<T> a, int root, int n, TComparer cmp) where TComparer : struct, IComparer<T>
    {
        while (true)
        {
            int child = 2 * root + 1;
            if (child >= n) { return; }
            if (child + 1 < n && cmp.Compare(a[child], a[child + 1]) < 0) { child++; }
            if (cmp.Compare(a[root], a[child]) >= 0) { return; }
            (a[root], a[child]) = (a[child], a[root]);
            root = child;
        }
    }
}
