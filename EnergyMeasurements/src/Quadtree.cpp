
/* Neocortex
 *
 * Author: Jarkko Vatjus-Anttila <jvatjusanttila@gmail.com>
 *
 * For conditions of distribution and use, see copyright notice in license.txt
 */

#include "Quadtree.h"
#include "DebugLog.h"
#include "Profiler.h"

#include <assert.h>

/******************************************************************************
 * Quadtree construction and destruction
 */

Quadtree::Quadtree(Scenegraph *p)
{
    parent = p;
    memset(qtree, 0, sizeof(qtree));
    _initRootnode();
}

Quadtree::~Quadtree()
{
    DebugLog::Instance()->MESSAGE(5, "Quadtree object destruction\n");
}

/******************************************************************************
 * Quadtree private API
 */

void Quadtree::_initRootnode(void)
{
    QUADTREE_NODE *root;
    root = & qtree[0];
    root->parent = QTREE_PARENT_INVALID;
    root->mode   = QTREE_MODE_POPULATED;
    root->min_x  = QTREE_MINX;
    root->max_x  = QTREE_MAXX;
    root->min_y  = QTREE_MINY;
    root->max_y  = QTREE_MAXY;
}

int Quadtree::_splitNode(NODEINDEX n)
{
    QUADTREE_NODE *rnode;
    QUADTREE_NODE *node1;
    QUADTREE_NODE *node2;
    QUADTREE_NODE *node3;
    QUADTREE_NODE *node4;

    assert(freeindex+4 <= QTREE_SIZE);

    rnode = & qtree[n];
    node1 = & qtree[freeindex++];
    node2 = & qtree[freeindex++];
    node3 = & qtree[freeindex++];
    node4 = & qtree[freeindex++];

    rnode->mode = QTREE_MODE_CHILDREN;

    node1->parent = n;
    node1->mode   = QTREE_MODE_POPULATED;
    node1->min_x  = rnode->min_x;
    node1->max_x  = (rnode->min_x + rnode->max_x) / 2.0f;
    node1->min_y  = rnode->min_y;
    node1->max_y  = (rnode->min_y + rnode->max_y) / 2.0f;

    node2->parent = n;
    node2->mode   = QTREE_MODE_POPULATED;
    node2->min_x  = (rnode->min_x + rnode->max_x) / 2.0f;
    node2->max_x  = rnode->max_x;
    node2->min_y  = rnode->min_y;
    node2->max_y  = (rnode->min_y + rnode->max_y) / 2.0f;

    node3->parent = n;
    node3->mode   = QTREE_MODE_POPULATED;
    node3->min_x  = rnode->min_x;
    node3->max_x  = (rnode->min_x + rnode->max_x) / 2.0f;
    node3->min_y  = (rnode->min_y + rnode->max_y) / 2.0f;
    node3->max_y  = rnode->max_y;

    node4->parent = n;
    node4->mode   = QTREE_MODE_POPULATED;
    node4->min_x  = (rnode->min_x + rnode->max_x) / 2.0f;
    node4->max_x  = rnode->max_x;
    node4->min_y  = (rnode->min_y + rnode->max_y) / 2.0f;
    node4->max_y  = rnode->max_y;

    foreach(OBJECTINDEX index, rnode->u.pop.population)
    {
        DebugLog::Instance()->MESSAGE(4, "Quadtree split: repopulating object id %d\n", index);
    }

    return 0;
}

NODEINDEX Quadtree::_traverseTop(float x, float y, float z)
{
    return _traverse(0, x, y, z);
}

NODEINDEX Quadtree::_traverse(NODEINDEX i, float x, float y, float z)
{
    QTREE_NODE *n;
    n = & qtree[i];

    DebugLog::Instance()->MESSAGE(4, "Traversing from nodeindex %d - %f %f %f\n", i, x, y, z);

    if ((x >= n->min_x) && (x < n->max_x) &&
        (y >= n->min_y) && (y < n->max_y))
    {
        if (n->mode == QTREE_MODE_CHILDREN)
        {
            foreach(NODEINDEX index, n->u.children.nodes)
            {
                QTREE_NODE *nextnode = & qtree[index];
                if ((x >= nextnode->min_x) && (x < nextnode->max_x) &&
                    (y >= nextnode->min_y) && (y < nextnode->max_y))
                {
                    return _traverse(index, x, y, z);
                }
            }
            DebugLog::Instance()->MESSAGE(4, "Quadtree childlist traversal failed\n");
            assert(0);
        }
        else if (n->mode == QTREE_MODE_POPULATED)
        {
            if (n->u.pop.n == QTREE_MAX_POPULATION-1)
            {
                DebugLog::Instance()->MESSAGE(4, "Quadtree node full. Making a split\n");
                _splitNode(i);
                return _traverse(i, x, y, z);
            }
            else
            {
                DebugLog::Instance()->MESSAGE(4, "Quadtree traverse found suitable node at %d\n", i);
                return i;
            }
        }
        else
        {
            DebugLog::Instance()->MESSAGE(4, "Quadtree node mode is invalid\n");
            assert(0);
        }
    }
    else
    {
        // Are we at the root node?
        assert(n->parent == QTREE_PARENT_INVALID);
        DebugLog::Instance()->MESSAGE(4, "Traverse lookup failed with %f %f %f\n", x, y, z);
        return _traverse(n->parent, x, y, z);
    }
}

int Quadtree::insertObject(NODEINDEX i, OBJECTINDEX id)
{
    QUADTREE_NODE *node = & qtree[i];
    node->u.population.population[node->u.population.n] = id;
    node->u.population.n += 1;
    return 0;
}

NODEINDEX Quadtree::queryObject()

/******************************************************************************
 * Quadtree API
 */

int Quadtree::addObject(OBJECTINDEX id)
{
    float x, y, z;
    NODEINDEX nodeindex;

#if !defined(UNIT_TEST)
    if (0 != parent->getObjectPosition(id, &z, &y, &z))
    {
        DebugLog::Instance()->MESSAGE(4, "Object position fetch failed when adding to quadtree\n");
        return -1;
    }
#else
    x = 0.5f;
    y = 0.25f;
    z = -1.5f;
#endif

    nodeindex = _traverseTop(x, y, z);
    assert(nodeindex != -1);
    return _insertObject(nodeindex, id);
}

#if defined(UNIT_TEST)
#define OBJ (5)
int main(int argc, char *args[])
{
    Quadtree qTree = new Quadtree;
    NODEINDEX k;

    DebugLog::Instance()->setVerbosityLevel(5);
    DebugLog::Instance()->MESSAGE(1, "sizeof(QUADTREE_NODE) = %d\n", sizeof(QUADTREE_NODE));

    for (int i; i<OBJ; i++)
    {
        Profiler::Instance()->startBlock("Quadtree::addObject");
        k = qTree->addObject((OBJECTINDEX)i);
        qTree->insertObject(k, i);
        Profiler::Instance()->endBlock();
    }

    for (int i; i<OBJ; i++)
    {
        Profiler::Instance()->startBlock("Quadtree::queryObject");
        k = qTree->queryObject((OBJECTINDEX)i);
        Profiler::Instance()->endBlock();
    }

    for (int i; i<OBJ; i++)
    {
        Profiler::Instance()->startBlock("Quadtree::updateObject");
        qTree->updateObject((OBJECTINDEX)i);
        Profiler::Instance()->endBlock();
    }
}
#endif

