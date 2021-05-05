#ifndef INSEG_LIB_DISJOINT_SET_H_
#define INSEG_LIB_DISJOINT_SET_H_

#include <vector>

namespace inseg_lib {
    
// Simple implementation of disjoint-set data structure.
class DisjointSet {
public:
    DisjointSet() = default;
    
    explicit DisjointSet(const int size);
    // Resizes the set and initializes memory.
    void Resize(const int size);
    
    // Finds an item.
    int Find(const int x);
    // Finds a set.
    bool FindSet(const int x, const int y);
    // Combines two sets.
    bool UnionSet(const int x, const int y);
    int GetSize(const int x);
    // Get size of the disjoint set which is the number of root elements.
    int GetTotalSize();
private:
    std::vector<int> parents_;
    // This tells us how many elements are in the disjoint set which is the total
    // size. It is set to the size of the root elements in the beginning and
    // decreased whenever a set it combined.
    int num_elements_ = 0;
};
}  // namespace inseg_lib

#endif  // INSEG_LIB_DISJOINT_SET_H_
