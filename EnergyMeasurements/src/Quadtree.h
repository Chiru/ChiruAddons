
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#ifndef Quadtree_H
#define Quadtree_H

#include <vector>

/// static quadtree size variables
#define QTREE_LEVELS (5)
#define QTREE_SIZE   (1 + 4 + 4*4 + 4*4*4 + 4*4*4*4)

/// Quadtree mode
#define QTREE_MODE_CHILDREN  (1)
#define QTREE_MODE_POPULATED (2)

#define QTREE_PARENT_INVALID (-1)
#define QTREE_OBJECTINDEX_INVALID (-1)
#define QTREE_NODEINDEX_INVALID (-1)

#define QTREE_MAX_POPULATION (8)

#define QTREE_MINX (-1.0f)
#define QTREE_MAXX  (1.0f)
#define QTREE_MINY (-1.0f)
#define QTREE_MAXY  (1.0f)

typedef short NODEINDEX;
typedef NODEINDEX OBJECTINDEX;


/// Quadtree data structures
typedef struct _quadtree_node {
    short mode;
    NODEINDEX parent;
    float min_x, max_x, min_y, max_y;
    union {
        struct {
            NODEINDEX nodes[4];
        } chi;
        struct {
            short n;
            OBJECTINDEX population[QTREE_MAX_POPULATION];
        } pop;
    } u;
} QUADTREE_NODE;


/// Class definition of Quadtree_Node
class Quadtree_Node
{
public:
    Quadtree_Node();
    ~Quadtree_Node();

private:
};

class Scenegraph;

/// Class definition
class Quadtree
{
public:
    Quadtree(Scenegraph *p);
    ~Quadtree();

    int addObject(OBJECTINDEX id);
    int removeObject(OBJECTINDEX id);
    int updateObject(OBJECTINDEX id);

protected:

private:
    Scenegraph *parent;

    QUADTREE_NODE qtree[QTREE_SIZE];
    NODEINDEX freeindex;

    /// Private member methods:
    void _initRootnode(void);
    NODEINDEX _traverse(NODEINDEX i, float x, float y, float z);
    NODEINDEX _traverseTop(void);
    NODEINDEX _traverseBottom(NODEINDEX node);
    int _splitNode(NODEINDEX n);

    unsigned int rootnode;
    std::vector <Quadtree_Node> nodes;
};


#endif // Quadtree_H
