/**
 * @file minheap.h
 * @brief Implements a Min-Heap data structure for the planner's frontier.
 *
 * This file provides an efficient implementation of a priority queue using a
 * binary min-heap. The frontier of the search algorithm (the open set) is
 * stored in this data structure, which allows for logarithmic time complexity
 * for insertions and extractions of the node with the minimum f-value.
 * The heap is also dynamically resizable to handle a large number of nodes.
 */

#ifndef MINHEAP_H
#define MINHEAP_H

#include <stdio.h>
#include <stdlib.h>

#include "auxiliary.h"

/**
 * @struct HeapNode
 * @brief Represents a single element within the Min-Heap.
 */
typedef struct {
    int f;          // The f-value (priority) of the search tree node.
    void *node;     // A void pointer to the actual search tree node (struct tree_node).
} HeapNode;

/**
 * @struct MinHeap
 * @brief The main Min-Heap data structure.
 */
typedef struct {
    HeapNode *nodeArray; // A dynamic array to store the heap elements.
    int nodeSize;        // The current number of elements in the heap.
    int capacity;        // The current allocated capacity of the array.
} MinHeap;

/**
 * @brief Creates and initializes a new Min-Heap.
 * @param capacity The initial capacity of the heap.
 * @return A pointer to the newly created MinHeap.
 */
MinHeap* createMinHeap(int capacity) {
    MinHeap *heap = (MinHeap*) malloc(sizeof(MinHeap));
    heap->nodeArray = (HeapNode*) malloc(capacity * sizeof(HeapNode));
    heap->nodeSize = 0;
    heap->capacity = capacity;
    return heap;
}

/**
 * @brief Doubles the capacity of the heap's internal array when it's full.
 * @param heap The heap to resize.
 */
void resizeMinHeap(MinHeap *heap) {
    heap->capacity *= 2;
    heap->nodeArray = (HeapNode*) realloc(heap->nodeArray, heap->capacity * sizeof(HeapNode));
    if (!heap->nodeArray) {
        printf("[ERROR] Memory allocation failed during heap resize!\n");
        exit(1);
    }
}

/**
 * @brief Swaps two HeapNode elements. A helper function for heap operations.
 * @param a Pointer to the first node.
 * @param b Pointer to the second node.
 */
void swapNodes(HeapNode *a, HeapNode *b) {
    HeapNode temp = *a;
    *a = *b;
    *b = temp;
}

/**
 * @brief Maintains the min-heap property starting from a given index.
 *
 * This function assumes the subtrees are already min-heaps. It lets the value
 * at index `i` "float down" the heap to its correct position.
 * @param heap The heap to be heapified.
 * @param i The index to start the heapify process from.
 */
void minHeapify(MinHeap *heap, int i) {
    int smallest;
    while (1) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        smallest = i;

        if (left < heap->nodeSize && heap->nodeArray[left].f < heap->nodeArray[smallest].f)
            smallest = left;
        if (right < heap->nodeSize && heap->nodeArray[right].f < heap->nodeArray[smallest].f)
            smallest = right;
        if (smallest == i) break; // The node is in its correct position.

        swapNodes(&heap->nodeArray[i], &heap->nodeArray[smallest]);
        i = smallest; // Move down to the next level.
    }
}

/**
 * @brief Inserts a new search node into the Min-Heap.
 *
 * Adds the node to the end of the array and then lets it "bubble up" to maintain
 * the min-heap property. Resizes the heap if necessary.
 * @param heap The heap to insert into.
 * @param f The f-value (priority) of the node.
 * @param node A void pointer to the search tree node.
 */
void insert_node(MinHeap *heap, int f, void *node) {
    if (heap->nodeSize == heap->capacity) {
        resizeMinHeap(heap);
    }

    // Insert the new node at the end.
    int i = heap->nodeSize;
    heap->nodeArray[i].f = f;
    heap->nodeArray[i].node = node;
    heap->nodeSize++;

    // Fix the min-heap property by bubbling the new node up.
    while (i > 0 && heap->nodeArray[(i - 1) / 2].f > heap->nodeArray[i].f) {
        swapNodes(&heap->nodeArray[i], &heap->nodeArray[(i - 1) / 2]);
        i = (i - 1) / 2;
    }

    total_inserts++; // For statistics.
}

/**
 * @brief Extracts the node with the minimum f-value from the heap.
 *
 * The root of the min-heap always contains the element with the highest priority
 * (lowest f-value). This function returns the root, replaces it with the last
 * element, and then calls minHeapify to restore the heap property.
 * @param heap The heap from which to extract the minimum element.
 * @return The HeapNode with the minimum f-value.
 */
HeapNode extract_min(MinHeap *heap) {
    if (heap->nodeSize == 0) {
        HeapNode empty_heap = {-1, NULL}; // Return an empty node if heap is empty.
        return empty_heap;
    }

    // The root is the minimum element.
    HeapNode root = heap->nodeArray[0];

    // Replace the root with the last element.
    heap->nodeArray[0] = heap->nodeArray[heap->nodeSize - 1];
    heap->nodeSize--;

    // Restore the min-heap property.
    minHeapify(heap, 0);

    return root;
}

/**
 * @brief Checks if the heap is empty.
 * @param heap The heap to check.
 * @return 1 if empty, 0 otherwise.
 */
int is_empty_heap(MinHeap *heap) {
    return heap->nodeSize == 0;
}

#endif // MINHEAP_H
