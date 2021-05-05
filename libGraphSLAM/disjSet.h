//
// Created by sc on 10/21/20.
//

#ifndef GRAPHSLAM_DISJSET_H
#define GRAPHSLAM_DISJSET_H
#include <map>
#include <sstream>
#include <set>
#include <cassert>
#include <vector>
#include <iostream>

namespace PSLAM{

    /**
@brief  A disjoint set forest is a fairly standard data structure used to represent the partition of
        a set of elements into disjoint sets in such a way that common operations such as merging two
        sets together are computationally efficient.

This implementation uses the well-known union-by-rank and path compression optimizations, which together
yield an amortised complexity for key operations of O(a(n)), where a is the (extremely slow-growing)
inverse of the Ackermann function.

The implementation also allows clients to attach arbitrary data to each element, which can be useful for
some algorithms.

@tparam T   The type of data to attach to each element (arbitrary)
*/
    template <typename T>
    class DisjointSetForest
    {
        //#################### NESTED CLASSES ####################
    private:
        struct Element
        {
            T m_value;
            int m_parent;
            int m_rank;

            Element(const T& value, int parent)
                    :   m_value(value), m_parent(parent), m_rank(0)
            {}
        };

        //#################### PRIVATE VARIABLES ####################
    private:
        mutable std::map<int,Element> m_elements;
        int m_setCount;

        //#################### CONSTRUCTORS ####################
    public:
        /**
        @brief  Constructs an empty disjoint set forest.
        */
        DisjointSetForest()
                :   m_setCount(0)
        {}

        /**
        @brief  Constructs a disjoint set forest from an initial set of elements and their associated values.

        @param[in]  initialElements     A map from the initial elements to their associated values
        */
        explicit DisjointSetForest(const std::map<int,T>& initialElements)
                :   m_setCount(0)
        {
            add_elements(initialElements);
        }

        //#################### PUBLIC METHODS ####################
    public:
        /**
        @brief  Adds a single element x (and its associated value) to the disjoint set forest.

        @param[in]  x       The index of the element
        @param[in]  value   The value to initially associate with the element
        @pre
            -   x must not already be in the disjoint set forest
        */
        void add_element(int x, const T& value = T())
        {
            m_elements.insert(std::make_pair(x, Element(value, x)));
            ++m_setCount;
        }

        std::set<int> find_nodes_on_same_set(int set) {
            std::set<int> indices;
            // find all indices that belong to this set
            for(auto &element :m_elements) {
                if(find_set(element.first)==set){
                    indices.insert(element.first);
                }
            }
            return indices;
        }

        void remove_element(int x)
        {
            // collect the groups that within the same set
            int set = find_set(x);
            std::set<int> related_set = find_nodes_on_same_set(set);

            DisjointSetForest<int> tmp;
            for(auto& idx:related_set) {
                if(idx==x)continue;
                if(tmp.is_element_exist(idx))continue;

                auto i = idx;
                tmp.add_element(i);
                std::cout << "strating element " << idx << "\n";
                while(true) {
                    auto parent = m_elements.at(i).m_parent;
                    if (parent == i) break;
                    if (parent == x) break; // should not go through the deleted node
                    tmp.add_element(parent);
                    tmp.union_sets(i,parent);
                    std::cout << "\tunionset of " << i << ", " << parent << "\n";
                    i = parent;
                }
            }
            auto related_nodes = tmp.get_element_keys();
            for(auto key : related_nodes) {
                std::cout << key << ": " << tmp.find_set(key) << ", " << tmp.rank_of(key) << "\n";
                m_elements.at(key).m_parent = tmp.find_set(key);
            }
//
//            // find who uses this as parent.
//            std::set<int> this_set;
//            int highest_rank=0;
//            int highest_idx =0;
//            for (const std::pair<int,Element> &element:m_elements){
//                if(element.second.m_parent == x) {
//                    this_set.insert(element.first);
//                    if(element.second.m_rank>highest_rank){
//                        highest_rank = element.second.m_rank;
//                        highest_idx = element.first;
//                    }
//                }
//            }
//            if(this_set.empty())--m_setCount;
//            else {
//                for(auto idx : this_set) {
//                    m_elements.at(idx).m_parent = idx;
//                }
//            }
            m_elements.erase(x);
        }

        /**
        @brief  Adds multiple elements (and their associated values) to the disjoint set forest.

        @param[in]  elements    A map from the elements to add to their associated values
        @pre
            -   None of the elements to be added must already be in the disjoint set forest
        */
        void add_elements(const std::map<int,T>& elements)
        {
            for(typename std::map<int,T>::const_iterator it=elements.begin(), iend=elements.end(); it!=iend; ++it)
            {
                m_elements.insert(std::make_pair(it->first, Element(it->second, it->first)));
            }
            m_setCount += elements.size();
        }

        /**
        @brief  Returns the number of elements in the disjoint set forest.

        @return As described
        */
        int element_count() const
        {
            return static_cast<int>(m_elements.size());
        }

        /**
        @brief  Finds the index of the root element of the tree containing x in the disjoint set forest.

        @param[in]  x   The element whose set to determine
        @pre
            -   x must be an element in the disjoint set forest
        @throw Exception
            -   If the precondition is violated
        @return As described
        */
        int find_set(int x) const
        {
            Element& element = get_element(x);
            int& parent = element.m_parent;
            if(parent != x)
            {
                parent = find_set(parent);
            }
            return parent;
        }

        /**
        @brief  Returns the current number of disjoint sets in the forest (i.e. the current number of trees).

        @return As described
        */
        int set_count() const
        {
            return m_setCount;
        }

        /**
        @brief  Merges the disjoint sets containing elements x and y.

        If both elements are already in the same disjoint set, this is a no-op.

        @param[in]  x   The first element
        @param[in]  y   The second element
        @pre
            -   Both x and y must be elements in the disjoint set forest
        @throw Exception
            -   If the precondition is violated
        */
        void union_sets(int x, int y)
        {
            int setX = find_set(x);
            int setY = find_set(y);
            if(setX != setY) link(setX, setY);
        }


        void remove_union(int x, int y)
        {
            int setX = find_set(x);
            int setY = find_set(y);
            if(setX == setY) unlink(setX, setY);
        }

        /**
        @brief  Returns the value associated with element x.

        @param[in]  x   The element whose value to return
        @pre
            -   x must be an element in the disjoint set forest
        @throw Exception
            -   If the precondition is violated
        @return As described
        */
        T& value_of(int x)
        {
            return get_element(x).m_value;
        }

        int rank_of(int x)
        {
            return get_element(x).m_rank;
        }

        /**
        @brief  Returns the value associated with element x.

        @param[in]  x   The element whose value to return
        @pre
            -   x must be an element in the disjoint set forest
        @throw Exception
            -   If the precondition is violated
        @return As described
        */
        const T& value_of(int x) const
        {
            return get_element(x).m_value;
        }


        bool is_element_exist(int x) const {
            return m_elements.find(x) != m_elements.end();
        }

        std::set<int> get_element_keys(){
            std::set<int> keys;
            for(auto pair:m_elements)keys.insert(pair.first);
            return keys;
        }
        //#################### PRIVATE METHODS ####################
    private:
        Element& get_element(int x) const
        {
            typename std::map<int,Element>::iterator it = m_elements.find(x);
            if(it != m_elements.end()) return it->second;
            else {
                std::stringstream ss;
                ss << "No such element: " << x;
                throw std::runtime_error( ss.str());
            }
        }

        void link(int x, int y)
        {
            Element& elementX = get_element(x);
            Element& elementY = get_element(y);
            int& rankX = elementX.m_rank;
            int& rankY = elementY.m_rank;
            if(rankX > rankY)
            {
                elementY.m_parent = x;
            }
            else
            {
                elementX.m_parent = y;
                if(rankX == rankY) ++rankY;
            }
            --m_setCount;
        }


        void find_set_tree(int x) {

        }

        void unlink(int x,int y)
        {
            int set = find_set(x);
            // need to find the group of them
            std::set<int> indices;
            // find all indices that belong to this set
            for(auto &element :m_elements) {
                if(find_set(element.first)==set){
                    indices.insert(element.first);
                }
            }
            // split them to the set that pass through x or y
            std::set<int> set_x, set_y;
            int top_rank_x=0, top_rank_y=0;
            int top_idx_x=0, top_idx_y=0;

            for(auto& idx:indices) {
                auto i = idx;
                while(true) {
                    auto &parent = m_elements.at(i).m_parent;
                    if (parent == set) break;  // stop when going to the top most
                    if (parent == x) {
                        set_x.insert(i);
                        if(m_elements.at(i).m_rank>top_rank_x) {
                            top_rank_x = m_elements.at(i).m_rank;
                            top_idx_x = i;
                        }
                        break;
                    }
                    if (parent == y) {
                        set_y.insert(i);
                        if(m_elements.at(i).m_rank>top_rank_y){
                            top_rank_y = m_elements.at(i).m_rank;
                            top_idx_y = i;
                        }
                        break;
                    }
                    i = parent;
                }
            }

            assert( top_idx_x != x );
            assert( top_idx_y != y );
            set_x.insert(x);
            set_y.insert(y);
            for(auto &s_x:set_x){
                m_elements.at(s_x).m_parent = top_idx_x;
            }
            for(auto &s_y:set_y){
                m_elements.at(s_y).m_parent = top_idx_y;
            }




        }

    };
}

#endif //GRAPHSLAM_DISJSET_H
